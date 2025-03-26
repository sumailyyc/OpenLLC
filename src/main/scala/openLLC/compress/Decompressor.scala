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
import DataTransformer._

class DecompressorIO[T <: Data](private val gen: T) extends Bundle {
  // Compressed input data
  val in     = Flipped(ValidIO(gen.cloneType))

  // final output data
  val out    = ValidIO(gen.cloneType)
}

abstract class Decompressor[T <: Data](val gen: T) extends Module {
  val io = IO(new DecompressorIO[T](gen))

  def decompress(compressedData: Valid[T]): Valid[T]

  io.out := decompress(io.in)
}

class ZeroValueDecompressor(gen: UInt, val itemSize: Int) extends Decompressor[UInt](gen) {

  assert(isPow2(itemSize), "Illegal item size")

  def decompress(compressedData: Valid[UInt]): Valid[UInt] = {
    val dataWidth = compressedData.bits.getWidth
    assert(dataWidth % itemSize == 0, "Illegal data length")

    val numItems = dataWidth / itemSize
    val combined = compressedData.bits ## 0.U(numItems.W)
    val mask = combined(combined.getWidth - 1, dataWidth)
    val data = combined(dataWidth - 1, 0)

    val maskVec = mask.asBools
    val dataVec = Wire(Vec(numItems, UInt(itemSize.W)))
    dataVec.zipWithIndex.foreach { case (e, i) =>
      e :=  data(dataWidth - i * itemSize - 1, dataWidth - (i + 1) * itemSize)
    }
    val indexVec = (1 to numItems).map(i => PopCount(maskVec.take(i)) - 1.U)
    val decodeVec = maskVec.zipWithIndex.map { case (v, i) => Mux(v, dataVec(indexVec(i)), 0.U(itemSize.W))}
    
    val uncompressedData = Wire(chiselTypeOf(compressedData))
    uncompressedData.valid := compressedData.valid
    uncompressedData.bits := decodeVec.reduce(_ ## _)
    uncompressedData
  }
}

class DontDecompressor(gen: UInt) extends Decompressor[UInt](gen) {
  def decompress(compressedData: Valid[UInt]): Valid[UInt] = {
    compressedData
  }
}

class DGBDecompressor(gen: UInt) extends Decompressor[UInt](gen) {
  def decompress(compressedData: Valid[UInt]): Valid[UInt] = {
    val dataWidth = compressedData.bits.getWidth

    val combined = compressedData.bits ## 0.U(6.W)
    val header = combined(combined.getWidth - 1, dataWidth)
    val data = combined(dataWidth - 1, 0)
    val dgbId = header(2, 0)
    val bitPlaneEnable = header(3)
    val zvcId = header(5, 4)

    val zvcd_8b = Module(new ZeroValueDecompressor(data, 8))
    val zvcd_16b = Module(new ZeroValueDecompressor(data, 16))
    val zvcd_32b = Module(new ZeroValueDecompressor(data, 32))
    val zvcd_64b = Module(new ZeroValueDecompressor(data, 64))

    zvcd_8b.io.in.valid := compressedData.valid
    zvcd_16b.io.in.valid := compressedData.valid
    zvcd_32b.io.in.valid := compressedData.valid
    zvcd_64b.io.in.valid := compressedData.valid
    zvcd_8b.io.in.bits := data
    zvcd_16b.io.in.bits := data
    zvcd_32b.io.in.bits := data
    zvcd_64b.io.in.bits := data

    val zvcDecodeVec = Wire(Vec(4, chiselTypeOf(data)))
    zvcDecodeVec(0) := zvcd_8b.io.out.bits
    zvcDecodeVec(1) := zvcd_16b.io.out.bits
    zvcDecodeVec(2) := zvcd_32b.io.out.bits
    zvcDecodeVec(3) := zvcd_64b.io.out.bits

    val zvcDecode = zvcDecodeVec(zvcId)
    val dgbDecodeVec = Wire(Vec(5, chiselTypeOf(zvcDecode)))
    dgbDecodeVec(0) := zvcDecode
    dgbDecodeVec(1) := deltaGrayBitPlaneDecoding(zvcDecode, 8, bitPlaneEnable)
    dgbDecodeVec(2) := deltaGrayBitPlaneDecoding(zvcDecode, 16, bitPlaneEnable)
    dgbDecodeVec(3) := deltaGrayBitPlaneDecoding(zvcDecode, 32, bitPlaneEnable)
    dgbDecodeVec(4) := deltaGrayBitPlaneDecoding(zvcDecode, 64, bitPlaneEnable)

    val uncompressedData = Wire(chiselTypeOf(compressedData))
    uncompressedData.valid := compressedData.valid
    uncompressedData.bits := dgbDecodeVec(dgbId)
    uncompressedData
  }
}