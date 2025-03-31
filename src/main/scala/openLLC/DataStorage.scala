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
import coupledL2.utils.SRAMTemplate
import org.chipsalliance.cde.config.Parameters

class DSRequest(implicit p: Parameters) extends LLCBundle {
  val way = UInt(wayBits.W)
  val set = UInt(setBits.W)
}

class DSRead(implicit p: Parameters) extends DSRequest

class DSWrite(implicit p: Parameters) extends DSRequest {
  val writeLeft = if (cacheParams.enableCompression) Some(Bool()) else None
  val wSubBlocks = if (cacheParams.enableCompression) Some(UInt(log2Ceil(subBlocks + 1).W)) else None
}

class DSBeat(implicit p: Parameters) extends LLCBundle {
  val data = UInt((beatBytes * 8).W)
}

class DSBlock(implicit p: Parameters) extends LLCBundle {
  val data = Vec(beatSize, new DSBeat())
}

class WBEntry(implicit p: Parameters) extends LLCBundle {
  val blockIdx = UInt(blockBits.W)
  val data = new DSBlock
}

class DataStorage(implicit p: Parameters) extends LLCModule {
  val io = IO(new Bundle() {
    /**
      * Support read and write request in the same cycle.
      * When reading and writing the same address,
      * the data before writing is returned
      */
    val read  = Flipped(ValidIO(new DSRead()))
    val write = Flipped(ValidIO(new DSWrite()))
    val rdata = Output(new DSBlock())
    val wdata = Input(new DSBlock())
  })

  val array = Seq.fill(subBlocks)(Module(new SRAMTemplate(
    gen = UInt((subBlockBytes * 8).W),
    set = blocks,
    way = 1,
    singlePort = false
  )))

  val ren = io.read.valid
  val wen = io.write.valid
  val readIdx = Cat(io.read.bits.way, io.read.bits.set)
  val writeIdx = Cat(io.write.bits.way, io.write.bits.set)

  val writeBuffer = RegInit(0.U.asTypeOf(new WBEntry()))
  val wmaskReg = RegInit(VecInit(Seq.fill(subBlocks)(false.B)))

  val writeHit = writeIdx === writeBuffer.blockIdx
  val writeBack = !writeHit && wen

  /* WriteBuffer update logic */
  /**
    * New write requests are not written directly to SRAM,
    * but are written to the buffer first.
    */
  when (wen) {
    writeBuffer.blockIdx := writeIdx
    if (cacheParams.enableCompression) {
      val safeLen = (io.write.bits.wSubBlocks.get << log2Ceil(subBlockBytes * 8)).min((blockBytes * 8).U)
      val mask = Wire(UInt((blockBytes * 8).W))
      val ones = (1.U << safeLen) - 1.U
      mask := Mux(io.write.bits.writeLeft.get, ones << ((blockBytes * 8).U - safeLen), ones)
      val dataShift = Mux(io.write.bits.writeLeft.get, io.wdata.data.asUInt, io.wdata.data.asUInt >> ((blockBytes * 8).U - safeLen))
      val dataCat = (mask & dataShift) | (~mask & writeBuffer.data.data.asUInt)
      writeBuffer.data.data.zipWithIndex.foreach { case (data, i) =>
        val beat = Wire(new DSBeat())
        beat.data := dataCat(beatBytes * (i + 1) * 8 - 1, beatBytes * i * 8)
        data := beat
      }
      val maskZip = Wire(Vec(subBlocks, Bool()))
      maskZip.zipWithIndex.foreach { case (zip, i) => zip := mask(subBlockBytes * 8 * (i + 1) - 1, subBlockBytes * 8 * i) =/= 0.U }
      wmaskReg.zip(maskZip).foreach { case (s, t) => s := Mux(writeBack, t, s || t) }
    } else {
      wmaskReg.foreach(_ := true.B)
      writeBuffer.data := io.wdata
    }
  }

  /* SRAM write logic */
  // SRAM is written when the data block of the buffer is replaced
  val bufferReadVec = Wire(Vec(subBlocks, UInt((subBlockBytes * 8).W)))
  bufferReadVec.zipWithIndex.foreach { case (d, i) => d := writeBuffer.data.asUInt(subBlockBytes * 8 * (i + 1) - 1, subBlockBytes * 8 * i) }
  array.zipWithIndex.foreach { case (e, i) => e.io.w.apply(writeBack && wmaskReg(i), bufferReadVec(i), writeBuffer.blockIdx, 1.U) }

  /* Read request response */
  val readHit = readIdx === writeBuffer.blockIdx
  val readBuffer = readHit && ren
  array.foreach(_.io.r.apply(ren, readIdx))
  val arrayReadVec = Wire(Vec(subBlocks, UInt((subBlockBytes * 8).W)))
  arrayReadVec.zipWithIndex.foreach { case (r, i) => r := array(i).io.r.resp.data(0) }
  val arrayRead = Wire(new DSBlock)
  arrayRead.data.zipWithIndex.foreach { case (data, i) =>
    val beat = Wire(new DSBeat())
    beat.data := arrayReadVec.asUInt(beatBytes * (i + 1) * 8 - 1, beatBytes * i * 8)
    data := beat
  }
  val catReadVec = VecInit(bufferReadVec.zip(arrayReadVec).zip(wmaskReg).map { case ((buffer, array), mask) =>
    Mux(RegNext(mask, false.B), RegNext(buffer, 0.U), array) })
  val catRead = Wire(new DSBlock)
  catRead.data.zipWithIndex.foreach { case (data, i) =>
    val beat = Wire(new DSBeat())
    beat.data := catReadVec.asUInt(beatBytes * (i + 1) * 8 - 1, beatBytes * i * 8)
    data := beat
  }

  val rdata_s1 = Mux(
    RegNext(readBuffer, false.B), 
    catRead,
    arrayRead
  )
  val rdata_s2 = RegEnable(rdata_s1, 0.U.asTypeOf(new DSBlock), RegNext(ren, false.B))
  io.rdata := rdata_s2

}