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

package openLLC.compress

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters

/** IO bundle definition for an Compressor, which takes one valid input and outputs
  * the compressed data.
  *
  * @param gen data type
  */
class CompressorIO[T <: Data](private val gen: T) extends Bundle {
  // Uncompressed input data
  val in     = Flipped(Valid(gen.cloneType))

  // final output data
  val out    = Valid(gen.cloneType)

  // Indicate whether the output data is compressd or not
  val compressed = Output(Bool())

  // The actual length of the compressed data if compression is possible
  val length = Output(UInt(log2Ceil(gen.getWidth).W))
}

abstract class Compressor[T <: Data](val gen: T) extends Module {
  val io = IO(new CompressorIO[T](gen))

  def compress(rawData: Valid[T]): (Valid[T], Bool, UInt)

  val (result, compressed, length) = compress(io.in)

  io.out        := result
  io.length     := length
  io.compressed := compressed
}

class ZeroValueCompressor(gen: UInt, val itemSize: Int) extends Compressor[UInt](gen) {

  assert(isPow2(itemSize), "Illegal item size")

  def compress(rawData: Valid[UInt]): (Valid[UInt], Bool, UInt) = {
    val dataWidth = rawData.bits.getWidth
    assert(dataWidth % itemSize == 0, "Illegal data length")

    val numItems = dataWidth / itemSize
    val itemVec = Wire(Vec(numItems, UInt(itemSize.W)))
    itemVec.zipWithIndex.foreach { case (e, i) =>
      e :=  rawData.bits(dataWidth - i * itemSize - 1, dataWidth - (i + 1) * itemSize)
    }
    val maskVec = itemVec.map(_ =/= 0.U)
    val popCountVec = (1 to numItems).map(i => PopCount((maskVec.take(i))))
    val indexVec = (1 to numItems).map(i => PriorityEncoder(popCountVec.map(_ === i.U)))
    val nonZeroVec = indexVec.map(itemVec(_))

    val totalLen = (PopCount(maskVec) << log2Ceil(itemSize)) +& maskVec.size.U
    val canCompress = totalLen < dataWidth.U
    val compressedData = Wire(chiselTypeOf(rawData))
    compressedData.valid := rawData.valid
    when (canCompress) {
      val itemConcat = nonZeroVec.reduce(_ ## _)
      val maskConcat = VecInit(maskVec).asUInt
      val combined = maskConcat ## itemConcat
      compressedData.bits := combined(combined.getWidth - 1, maskConcat.getWidth)
    }.otherwise {
      compressedData.bits := rawData.bits
    }
    (compressedData, canCompress, totalLen)
  }
}

class DontCompressor(gen: UInt) extends Compressor[UInt](gen) {
  def compress(rawData: Valid[UInt]): (Valid[UInt], Bool, UInt) = {
    (rawData, false.B, gen.getWidth.U)
  }
}

/* class DGBCompressor(val gen: DSBlock, val n: Int) extends Compressor[DSBlock](gen, n) {
  private def DGB(rawData: UInt, itemSize: Int): UInt = {
    assert(rawData.getWidth % itemSize == 0, "Illegal data length")
    val numItems = rawData.getWidth / itemSize
    val itemVec = Wire(Vec(numItems, UInt(itemSize.W)))
    itemVec.zipWithIndex.foreach { case (e, i) =>
      e :=  rawData((i + 1) * itemSize - 1, i * itemSize)
    }
    def deltaEncoding(dataVec: Vec[UInt]): Vec[UInt] = {
      val encodingVec = Wire(chiselTypeOf(dataVec))
      val baseValue = dataVec.head
      encodingVec.head := baseValue
      encodingVec.tail.zip.dataVec.tail.foreach { case (sink, src) =>
        sink := src - baseValue
      }
      encodingVec
    }
    def grayEncoding(data: UInt): UInt = data ^ (data >> 1.U)
    def bitPlaneEncoding(dataVec: Vec[UInt]): Vec[UInt] = {
      val rows = dataVec.size
      val cols = dataVec.head.getWidth
      val encodingVec = Wire(Vec(cols, UInt(rows.W)))
      encodingVec.zipWithIndex.foreach { case (e, i) =>
        e := Cat(dataVec.map(t => t(i)))
      }
      encodingVec
    }
    bitPlaneEncoding(VecInit(deltaEncoding(itemVec).map(_ => grayEncoding(_))))
  }
  
  def compress(in: Valid[DSBlock]) = {

  }
} */
