// Copyright Juan Schroeder
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors:
// - Juan Schroeder  <jcschroeder@gmail.com>

/// Contains desc64 extra AXI AXI Stream definitions
package idma_desc64_axi_axis_pkg;

  /// Used for the completion metadata of a desc64 transfer.
  typedef struct packed {
    logic unsigned do_irq;
    logic unsigned preserve_desc;
  } idma_req_metadata_t;

endpackage