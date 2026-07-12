// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Axel Vanoni <axvanoni@ethz.ch>

`include "common_cells/registers.svh"

/// This module takes in an AXI R-channel, and reads descriptors from it.
/// Note that an using an address width other than 64 bits will need
/// modifications.
module idma_desc64_axi_axis_reader #(
    /// Address width of the AXI bus
    parameter int unsigned AddrWidth    = 64,
    /// Data width of the AXI bus
    parameter int unsigned DataWidth    = 64,
    /// iDMA request type
    parameter type         idma_req_t   = logic,
    /// AXI R channel type
    parameter type         axi_r_chan_t = logic,
    /// Configuration descriptor type
    parameter type         descriptor_t = logic,
    /// AXI bus address type, derived from the address width
    parameter type         addr_t       = logic [AddrWidth-1:0],
    /// Completion metadata
    parameter type         idma_req_metadata_t = logic
)(
    /// clock
    input  logic        clk_i,
    /// reset
    input  logic        rst_ni,
    /// axi read channel
    input  axi_r_chan_t r_chan_i,
    /// read channel valid
    input  logic        r_chan_valid_i,
    /// read channel ready
    output logic        r_chan_ready_o,
    /// idma request
    output idma_req_t   idma_req_o,
    /// idma request valid
    output logic        idma_req_valid_o,
    /// idma request ready
    /// NOTE: we assume that if a read was launched,
    /// the connected fifo has still space left, i.e. this signal is always
    /// 1 if a request is in-flight. If a request is in-flight and there
    /// is not enough space in the fifo, we will either stall the bus or
    /// drop the request.
    input  logic        idma_req_ready_i,
    /// location of the next descriptor address
    output addr_t       next_descriptor_addr_o,
    /// whether next_descriptor_addr is valid
    output logic        next_descriptor_addr_valid_o,
    /// idma request metadata
    output idma_req_metadata_t   idma_req_metadata_o,
    /// whether metadata is valid
    output logic        idma_req_metadata_valid_o,
    /// whether a request is in-flight
    output logic        idma_req_inflight_o
);

descriptor_t current_descriptor;
logic descriptor_valid;

if (DataWidth == 256) begin : gen_256_data_path
    assign current_descriptor           = r_chan_i.data;
    //assign idma_req_valid_o             = r_chan_valid_i;
    // no invalid backend request
    assign idma_req_valid_o             = r_chan_valid_i && descriptor_valid;
    //assign next_descriptor_addr_valid_o = r_chan_valid_i;
    // no next descriptor fetch
    assign next_descriptor_addr_valid_o = r_chan_valid_i && descriptor_valid;
    assign idma_req_inflight_o          = r_chan_valid_i;
end else if (DataWidth == 128) begin : gen_128_data_path
    logic [127:0] first_half_of_descriptor_q, first_half_of_descriptor_d;
    logic [127:0] second_half_of_descriptor;
    logic         metadata_first_half_valid_q, metadata_first_half_valid_d;

    //assign idma_req_valid_o             = r_chan_valid_i && r_chan_i.last;
    assign idma_req_valid_o             = r_chan_valid_i && r_chan_i.last && descriptor_valid;
    assign next_descriptor_addr_valid_o = metadata_first_half_valid_q;
    assign idma_req_inflight_o          = r_chan_valid_i || metadata_first_half_valid_q;

    assign current_descriptor = descriptor_t'{
        first_half_of_descriptor_q,
        second_half_of_descriptor
    };

    always_comb begin
        first_half_of_descriptor_d = first_half_of_descriptor_q;
        if (r_chan_valid_i && r_chan_ready_o && !r_chan_i.last) begin
            first_half_of_descriptor_d = r_chan_i.data;
        end
    end

    always_comb begin
        // The next-address field is valid
        // from receiving the first half until the
        // second half was received
        metadata_first_half_valid_d = metadata_first_half_valid_q;
        if (r_chan_valid_i && r_chan_ready_o) begin
            metadata_first_half_valid_d = !r_chan_i.last;
        end
    end

    `FF(first_half_of_descriptor_q, first_half_of_descriptor_d, 128'b0)
    `FF(metadata_first_half_valid_q, metadata_first_half_valid_d, 1'b0)
end else if (DataWidth == 64) begin : gen_64_data_path
    logic [1:0]       fetch_counter_q, fetch_counter_d;
    logic [2:0][63:0] descriptor_data_q, descriptor_data_d;
    logic [63:0]      descriptor_data_last;

    //assign idma_req_valid_o             = r_chan_valid_i && r_chan_i.last;
    assign idma_req_valid_o             = r_chan_valid_i && r_chan_i.last && descriptor_valid;
    assign next_descriptor_addr_valid_o = fetch_counter_q == 2'b10;
    assign descriptor_data_last         = r_chan_i.data;
    assign idma_req_inflight_o          = fetch_counter_q != 2'b00;

    assign current_descriptor = {
        descriptor_data_q[0],
        descriptor_data_q[1],
        descriptor_data_q[2],
        descriptor_data_last
    };

    always_comb begin : proc_fetch_data
        descriptor_data_d = descriptor_data_q;
        fetch_counter_d   = fetch_counter_q;
        if (r_chan_valid_i && r_chan_ready_o && !r_chan_i.last) begin
            descriptor_data_d[fetch_counter_q] = r_chan_i.data;
            fetch_counter_d                    = fetch_counter_q + 2'b01;
        end if (r_chan_valid_i && r_chan_i.last) begin
            fetch_counter_d                    = 2'b00;
        end
    end

    `FF(descriptor_data_q, descriptor_data_d, 192'b0)
    `FF(fetch_counter_q, fetch_counter_d, 2'b0)
end else if (DataWidth == 32) begin : gen_32_data_path
    logic [2:0]       fetch_counter_q, fetch_counter_d;
    logic [6:0][31:0] descriptor_data_q, descriptor_data_d;
    logic [31:0]      descriptor_data_last;

    //assign idma_req_valid_o             = r_chan_valid_i && r_chan_i.last;
    assign idma_req_valid_o             = r_chan_valid_i && r_chan_i.last && descriptor_valid;
    assign next_descriptor_addr_valid_o = fetch_counter_q == 3'd4;
    assign descriptor_data_last         = r_chan_i.data;
    assign idma_req_inflight_o          = fetch_counter_q != 3'd0;

    assign current_descriptor = {
        descriptor_data_q,
        descriptor_data_last
    };

    always_comb begin : proc_fetch_data
        descriptor_data_d = descriptor_data_q;
        fetch_counter_d   = fetch_counter_q;
        if (r_chan_valid_i && r_chan_ready_o && !r_chan_i.last) begin
            descriptor_data_d[fetch_counter_q] = r_chan_i.data;
            fetch_counter_d                    = fetch_counter_q + 3'b001;
        end if (r_chan_valid_i && r_chan_i.last) begin
            fetch_counter_d                    = 3'b0;
        end
    end
end

// Completion metadata must enter its FIFO in exact lockstep with the request
// FIFO.  Qualifying this with the request-ready handshake makes this a single
// event per accepted descriptor even if the AXI R channel inserts bubbles or
// holds the final beat under backpressure.
assign idma_req_metadata_valid_o = idma_req_valid_o && idma_req_ready_i;

//idma_desc64_reshaper #(
idma_desc64_axi_axis_reshaper #(
    .idma_req_t  (idma_req_t),
    .addr_t      (addr_t),
    .descriptor_t(descriptor_t),
    .idma_req_metadata_t(idma_req_metadata_t)
) i_descriptor_reshaper (
    .descriptor_i (current_descriptor),
    .descriptor_valid_o(descriptor_valid),
    .idma_req_o,
    .next_addr_o  (next_descriptor_addr_o),
    .idma_req_metadata_o
);

// The user should take care that the connected fifo always has
// enough space to put in the new descriptor. If it does not,
// instead of dropping requests, stall the bus (unless we're
// dropping this descriptor).
assign r_chan_ready_o               = idma_req_ready_i;

endmodule
