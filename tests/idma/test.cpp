// Copyright (C) 2026 Fondazione Chips-IT
//
// SPDX-License-Identifier: Apache-2.0

/*
 * Exercise the legacy iDMA exclusively through its public register, TCDM,
 * AXI, and completion-event interfaces. The AXI endpoint deliberately
 * completes write chunks out of issue order.
 *
 * The first two transfers are 48 and 64 bytes with a 32-byte TCDM width:
 *
 *   request order:  A32, A16, B32, B32
 *   response order: A32, B32, A16, B32
 *
 * The old AXI backend charges every response to pending_burst_ack.front().
 * After A32, A has 16 bytes remaining; charging B32 to it underflows that
 * counter and leaks the burst slot. Both slots eventually leak, so the third
 * transfer can never start and DONE_ID remains below its ticket.
 */

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <vp/itf/wire.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>


static constexpr uint64_t LOCAL_BASE = 0x10000000;
static constexpr uint64_t AXI_BASE = 0x20000000;
static constexpr uint64_t MEMORY_SIZE = 0x10000;

static constexpr uint64_t IDMA_CONF = 0x000;
static constexpr uint64_t IDMA_NEXT_ID = 0x00c;
static constexpr uint64_t IDMA_DONE_ID = 0x014;
static constexpr uint64_t IDMA_DST = 0x0d0;
static constexpr uint64_t IDMA_SRC = 0x0d8;
static constexpr uint64_t IDMA_LENGTH = 0x0e0;


class IDmaInterleavingTester : public vp::Component
{
public:
    IDmaInterleavingTester(vp::ComponentConf &config);
    void reset(bool active) override;

private:
    static vp::IoReqStatus tcdm_req(vp::Block *__this, vp::IoReq *req);
    static vp::IoReqStatus axi_req(vp::Block *__this, vp::IoReq *req);
    static void event_sync(vp::Block *__this, bool value);
    static void step_handler(vp::Block *__this, vp::ClockEvent *event);
    static void response_handler(vp::Block *__this, vp::ClockEvent *event);
    static void timeout_handler(vp::Block *__this, vp::ClockEvent *event);

    bool register_access(uint64_t offset, bool is_write, uint32_t &value);
    uint32_t submit(uint32_t src_offset, uint32_t dst_offset, uint32_t size);
    int next_response_index();
    void check_result();
    void fail(const char *reason);

    vp::IoMaster ctrl_itf;
    vp::IoSlave tcdm_itf;
    vp::IoSlave axi_itf;
    vp::WireSlave<bool> event_itf;
    vp::ClockEvent step_event;
    vp::ClockEvent response_event;
    vp::ClockEvent timeout_event;

    std::vector<uint8_t> local_memory;
    std::vector<uint8_t> axi_memory;
    std::vector<vp::IoReq *> pending_responses;
    std::vector<bool> response_sent;

    int forced_response_step = 0;
    int descriptors_submitted = 0;
    uint32_t last_done_id = 0;
    bool finished = false;
};


IDmaInterleavingTester::IDmaInterleavingTester(vp::ComponentConf &config)
    : vp::Component(config),
      step_event(this, &IDmaInterleavingTester::step_handler),
      response_event(this, &IDmaInterleavingTester::response_handler),
      timeout_event(this, &IDmaInterleavingTester::timeout_handler),
      local_memory(MEMORY_SIZE),
      axi_memory(MEMORY_SIZE, 0xa5)
{
    this->new_master_port("ctrl", &this->ctrl_itf);

    this->tcdm_itf.set_req_meth(&IDmaInterleavingTester::tcdm_req);
    this->new_slave_port("tcdm", &this->tcdm_itf, this);

    this->axi_itf.set_req_meth(&IDmaInterleavingTester::axi_req);
    this->new_slave_port("axi", &this->axi_itf, this);

    this->event_itf.set_sync_meth(&IDmaInterleavingTester::event_sync);
    this->new_slave_port("event", &this->event_itf, this);
}


void IDmaInterleavingTester::reset(bool active)
{
    if (!active)
    {
        for (uint64_t i = 0; i < this->local_memory.size(); ++i)
        {
            this->local_memory[i] = (uint8_t)((i * 37 + 11) & 0xff);
        }

        this->step_event.enqueue(1);
        this->timeout_event.enqueue(200);
    }
}


bool IDmaInterleavingTester::register_access(uint64_t offset, bool is_write, uint32_t &value)
{
    vp::IoReq req;
    req.prepare();
    req.set_addr(offset);
    req.set_size(sizeof(value));
    req.set_is_write(is_write);
    req.set_data((uint8_t *)&value);
    return this->ctrl_itf.req(&req) == vp::IO_REQ_OK;
}


uint32_t IDmaInterleavingTester::submit(
    uint32_t src_offset, uint32_t dst_offset, uint32_t size)
{
    uint32_t value = 0;
    if (!this->register_access(IDMA_CONF, true, value)) return 0;
    value = (uint32_t)(AXI_BASE + dst_offset);
    if (!this->register_access(IDMA_DST, true, value)) return 0;
    value = (uint32_t)(LOCAL_BASE + src_offset);
    if (!this->register_access(IDMA_SRC, true, value)) return 0;
    value = size;
    if (!this->register_access(IDMA_LENGTH, true, value)) return 0;
    value = 0;
    if (!this->register_access(IDMA_NEXT_ID, false, value)) return 0;
    return value;
}


vp::IoReqStatus IDmaInterleavingTester::tcdm_req(vp::Block *__this, vp::IoReq *req)
{
    IDmaInterleavingTester *_this = (IDmaInterleavingTester *)__this;
    uint64_t address = req->get_addr();
    uint64_t size = req->get_size();
    if (address + size > _this->local_memory.size()) return vp::IO_REQ_INVALID;

    if (req->get_is_write())
    {
        memcpy(&_this->local_memory[address], req->get_data(), size);
    }
    else
    {
        memcpy(req->get_data(), &_this->local_memory[address], size);
    }
    return vp::IO_REQ_OK;
}


vp::IoReqStatus IDmaInterleavingTester::axi_req(vp::Block *__this, vp::IoReq *req)
{
    IDmaInterleavingTester *_this = (IDmaInterleavingTester *)__this;
    if (!req->get_is_write() || req->get_addr() < AXI_BASE)
    {
        return vp::IO_REQ_INVALID;
    }

    uint64_t address = req->get_addr() - AXI_BASE;
    uint64_t size = req->get_size();
    if (address + size > _this->axi_memory.size()) return vp::IO_REQ_INVALID;

    // The write is accepted now, while its completion is deliberately delayed.
    memcpy(&_this->axi_memory[address], req->get_data(), size);
    _this->pending_responses.push_back(req);
    _this->response_sent.push_back(false);

    if (!_this->response_event.is_enqueued())
    {
        _this->response_event.enqueue(1);
    }
    return vp::IO_REQ_PENDING;
}


void IDmaInterleavingTester::event_sync(vp::Block *__this, bool value)
{
    // DONE_ID is the architectural completion state checked by this test.
    (void)__this;
    (void)value;
}


int IDmaInterleavingTester::next_response_index()
{
    // Wait until B's first chunk exists, then force A32, B32, A16.
    static constexpr int forced_order[] = {0, 2, 1};
    if (this->forced_response_step < 3)
    {
        int index = forced_order[this->forced_response_step];
        if (index >= (int)this->pending_responses.size()) return -1;
        this->forced_response_step++;
        return index;
    }

    // Thereafter drain every new request in issue order. With correct burst
    // accounting this includes B's tail followed by the third transfer.
    for (int i = 0; i < (int)this->pending_responses.size(); ++i)
    {
        if (!this->response_sent[i]) return i;
    }
    return -1;
}


void IDmaInterleavingTester::response_handler(vp::Block *__this, vp::ClockEvent *event)
{
    IDmaInterleavingTester *_this = (IDmaInterleavingTester *)__this;
    if (_this->finished) return;

    int index = _this->next_response_index();
    if (index >= 0)
    {
        vp::IoReq *req = _this->pending_responses[index];
        _this->response_sent[index] = true;
        req->get_resp_port()->resp(req);
    }

    _this->response_event.enqueue(1);
}


void IDmaInterleavingTester::step_handler(vp::Block *__this, vp::ClockEvent *event)
{
    IDmaInterleavingTester *_this = (IDmaInterleavingTester *)__this;
    if (_this->finished) return;

    if (_this->descriptors_submitted < 3)
    {
        static constexpr uint32_t src_offsets[] = {0, 64, 160};
        static constexpr uint32_t dst_offsets[] = {0, 64, 160};
        static constexpr uint32_t sizes[] = {48, 64, 32};

        int descriptor = _this->descriptors_submitted;
        uint32_t ticket = _this->submit(
            src_offsets[descriptor], dst_offsets[descriptor], sizes[descriptor]);
        uint32_t expected_ticket = descriptor + 1;
        if (ticket != expected_ticket)
        {
            char reason[128];
            snprintf(reason, sizeof(reason),
                "descriptor %d returned ticket %u instead of %u",
                descriptor, ticket, expected_ticket);
            _this->fail(reason);
            return;
        }
        _this->descriptors_submitted++;
    }

    uint32_t done_id = 0;
    if (!_this->register_access(IDMA_DONE_ID, false, done_id))
    {
        _this->fail("DONE_ID read failed");
        return;
    }
    _this->last_done_id = done_id;

    if (done_id >= 3)
    {
        _this->check_result();
        return;
    }

    _this->step_event.enqueue(1);
}


void IDmaInterleavingTester::check_result()
{
    static constexpr struct
    {
        uint32_t offset;
        uint32_t size;
    } transfers[] = {{0, 48}, {64, 64}, {160, 32}};

    for (const auto &transfer : transfers)
    {
        if (memcmp(&this->axi_memory[transfer.offset],
                   &this->local_memory[transfer.offset], transfer.size) != 0)
        {
            this->fail("destination data does not match the TCDM source");
            return;
        }
    }

    this->finished = true;
    printf("[%ld] tester PASS done_id=%u reordered_responses=1\n",
        this->clock.get_cycles(), this->last_done_id);
    this->time.get_engine()->quit(0);
}


void IDmaInterleavingTester::timeout_handler(vp::Block *__this, vp::ClockEvent *event)
{
    IDmaInterleavingTester *_this = (IDmaInterleavingTester *)__this;
    if (_this->finished) return;

    char reason[160];
    snprintf(reason, sizeof(reason),
        "timeout waiting for ticket 3 (done_id=%u, AXI requests=%zu)",
        _this->last_done_id, _this->pending_responses.size());
    _this->fail(reason);
}


void IDmaInterleavingTester::fail(const char *reason)
{
    if (this->finished) return;
    this->finished = true;
    printf("[%ld] tester FAIL %s\n", this->clock.get_cycles(), reason);
    this->time.get_engine()->quit(1);
}


extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new IDmaInterleavingTester(config);
}
