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