/** *************************************************************************************
 * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
 * Copyright (c) 2020-2021 Peng Cheng Laboratory
 *
 * XiangShan is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 * http://license.coscl.org.cn/MulanPSL2
 *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 *
 * See the Mulan PSL v2 for more details.
 * *************************************************************************************
 */

package openLLC

import chisel3._
import chisel3.util._
import coupledL2.tl2chi.CHIREQ
import freechips.rocketchip.amba.axi4.{AXI4BundleAR, AXI4BundleR}
import freechips.rocketchip.diplomacy._
import org.chipsalliance.cde.config.Parameters

class READ(implicit p: Parameters) extends LLCModule{
  val io = IO(new Bundle() {
    // downstream
    val ar = DecoupledIO(new AXI4BundleAR(edgeOut.bundle))
    val r = Flipped(new AXI4BundleR(edgeOut.bundle))

    // receive inner task
    val task = Flipped(DecoupledIO(new CHIREQ()))

    // response
    // val resp =
  })

  dontTouch(io)
}