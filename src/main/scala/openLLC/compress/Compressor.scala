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
import utility.{ParallelMax, ParallelMin}
import utility.XSPerfAccumulate

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

  // Indicate whether the output data is compressed or not
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

  require(isPow2(itemSize), "Illegal item size")

  def compress(rawData: Valid[UInt]): (Valid[UInt], Bool, UInt) = {
    val dataWidth = rawData.bits.getWidth
    require(dataWidth % itemSize == 0, "Illegal data length")

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
    assert(canCompress || rawData.bits === compressedData.bits)
    (compressedData, canCompress, totalLen)
  }
}

class DontCompressor(gen: UInt) extends Compressor[UInt](gen) {
  def compress(rawData: Valid[UInt]): (Valid[UInt], Bool, UInt) = {
    (rawData, false.B, (gen.getWidth - 1).U)
  }
}

object DataTransformer {

  def deltaEncoding(data: UInt, itemSize: Int): UInt = {
    require(isPow2(itemSize), "Illegal item size")
    require(data.getWidth % itemSize == 0, "Illegal data length")
    val numItems = data.getWidth / itemSize
    val itemVec = Wire(Vec(numItems, UInt(itemSize.W)))
    itemVec.zipWithIndex.foreach { case (e, i) =>
      e :=  data((i + 1) * itemSize - 1, i * itemSize)
    }
    val encodingVec = Wire(chiselTypeOf(itemVec))
    val baseValue = itemVec.head
    encodingVec.head := baseValue
    (encodingVec.tail zip itemVec.tail).foreach { case (sink, src) =>
      sink := src - baseValue
    }
    encodingVec.asUInt
  }

  def grayEncoding(data: UInt): UInt = data ^ (data >> 1.U)

  def bitPlaneEncoding(data: UInt, itemSize: Int): UInt = {
    val totalWidth = data.getWidth

    require(isPow2(itemSize), "Illegal item size")
    require(totalWidth % itemSize == 0, "Data width must be divisible by itemSize")

    val rows = totalWidth / itemSize
    val cols = itemSize

    val encodedBits = Seq.tabulate(cols) { col =>
      Seq.tabulate(rows) { row =>
        val bitPos = (rows - 1 - row) * cols + (cols - 1 - col)
        data(bitPos)
      }
    }.flatten

    Cat(encodedBits)
  }

  def deltaGrayBitPlaneEncoding(data: UInt, itemSize: Int): (UInt, UInt) = {
    require(isPow2(itemSize), "Illegal item size")
    require(data.getWidth % itemSize == 0, "Illegal data length")
    val deltaCode = deltaEncoding(data, itemSize)
    val numItems = deltaCode.getWidth / itemSize
    val itemVec = Wire(Vec(numItems, UInt(itemSize.W)))
    itemVec.zipWithIndex.foreach { case (e, i) =>
      e :=  deltaCode((i + 1) * itemSize - 1, i * itemSize)
    }
    val grayCode = VecInit(itemVec.map(grayEncoding(_))).asUInt
    (bitPlaneEncoding(grayCode, itemSize), grayCode)
  }

  def deltaDecoding(data: UInt, itemSize: Int): UInt = {
    require(isPow2(itemSize), "Illegal item size")
    require(data.getWidth % itemSize == 0, "Illegal data length")
    val numItems = data.getWidth / itemSize
    val itemVec = Wire(Vec(numItems, UInt(itemSize.W)))
    itemVec.zipWithIndex.foreach { case (e, i) =>
      e :=  data((i + 1) * itemSize - 1, i * itemSize)
    }
    val encodingVec = Wire(chiselTypeOf(itemVec))
    val baseValue = itemVec.head
    encodingVec.head := baseValue
    (encodingVec.tail zip itemVec.tail).foreach { case (sink, src) =>
      sink := src + baseValue
    }
    encodingVec.asUInt
  }

  def grayDecoding(data: UInt): UInt = {
    val width = data.getWidth
    val binary = Wire(Vec(width, Bool()))
    binary(width - 1) := data(width - 1)
    for (i <- (width-2) to 0 by -1) {
      binary(i) := data(i) ^ binary(i + 1)
    }
    binary.asUInt 
  }

  def bitPlaneDecoding(data: UInt, itemSize: Int): UInt = {
    val totalWidth = data.getWidth

    require(isPow2(itemSize), "Illegal item size")
    require(totalWidth % itemSize == 0, "Data width must be divisible by itemSize")

    val rows = totalWidth / itemSize
    val cols = itemSize

    val decodedBits = Seq.tabulate(rows) { row =>
      Seq.tabulate(cols) { col =>
        val bitPos = (cols - 1 - col) * rows + (rows - 1 - row)
        data(bitPos)
      }
    }.flatten

    Cat(decodedBits)
  }

  def deltaGrayBitPlaneDecoding(data: UInt, itemSize: Int, bitPlaneEnable: Bool): UInt = {
    require(isPow2(itemSize), "Illegal item size")
    require(data.getWidth % itemSize == 0, "Illegal data length")
    val bitPlaneDecode = bitPlaneDecoding(data, itemSize)
    val numItems = bitPlaneDecode.getWidth / itemSize
    val itemVec = Wire(Vec(numItems, UInt(itemSize.W)))
    itemVec.zipWithIndex.foreach { case (e, i) =>
      e :=  Mux(
        bitPlaneEnable,
        bitPlaneDecode((i + 1) * itemSize - 1, i * itemSize),
        data((i + 1) * itemSize - 1, i * itemSize)
      )
    }
    deltaDecoding(VecInit(itemVec.map(grayDecoding(_))).asUInt, itemSize)
  }
}

class DGBCompressor(gen: UInt)(implicit p: Parameters) extends Compressor[UInt](gen) {
  
  def compress(in: Valid[UInt]) = {
    val rawData = in.bits
    val dgb_8b = deltaGrayBitPlaneEncoding(rawData, 8)
    val dgb_16b = deltaGrayBitPlaneEncoding(rawData, 16)
    val dgb_32b = deltaGrayBitPlaneEncoding(rawData, 32)
    val dgb_64b = deltaGrayBitPlaneEncoding(rawData, 64)

    val encodeVec = Wire(Vec(9, chiselTypeOf(rawData)))
    encodeVec(0) := rawData
    encodeVec(1) := dgb_8b._1
    encodeVec(2) := dgb_8b._2
    encodeVec(3) := dgb_16b._1
    encodeVec(4) := dgb_16b._2
    encodeVec(5) := dgb_32b._1
    encodeVec(6) := dgb_32b._2
    encodeVec(7) := dgb_64b._1
    encodeVec(8) := dgb_64b._2

    val zeroItemCountVec = VecInit(encodeVec.map { e =>
      val numItems = e.getWidth / 8
      val vec = Wire(Vec(numItems, UInt(8.W)))
      for (i <- 0 until numItems) { vec(i) := e((i + 1) * 8 - 1, i * 8) }
      PopCount(vec.map(_ === 0.U))
    })
    val zeroItemCountMax = ParallelMax(zeroItemCountVec)
    val encodeId = PriorityEncoder(VecInit(zeroItemCountVec.map(_ === zeroItemCountMax)))
    val toZVC = encodeVec(encodeId)
    val dgbId = MuxLookup(encodeId, 0.U)(Seq(
      0.U -> 0.U,
      1.U -> 1.U,
      2.U -> 1.U,
      3.U -> 2.U,
      4.U -> 2.U,
      5.U -> 3.U,
      6.U -> 3.U,
      7.U -> 4.U,
      8.U -> 4.U,
    ))
    val bitPlaneEnable = encodeId(0)

    val zvc_8b = Module(new ZeroValueCompressor(rawData, 8))
    val zvc_16b = Module(new ZeroValueCompressor(rawData, 16))
    val zvc_32b = Module(new ZeroValueCompressor(rawData, 32))
    val zvc_64b = Module(new ZeroValueCompressor(rawData, 64))

    zvc_8b.io.in.valid := in.valid
    zvc_16b.io.in.valid := in.valid
    zvc_32b.io.in.valid := in.valid
    zvc_64b.io.in.valid := in.valid
    zvc_8b.io.in.bits := toZVC
    zvc_16b.io.in.bits := toZVC
    zvc_32b.io.in.bits := toZVC
    zvc_64b.io.in.bits := toZVC

    val dataVec = Wire(Vec(4, chiselTypeOf(zvc_8b.io.out.bits)))
    val compressedVec = Wire(Vec(4, chiselTypeOf(zvc_8b.io.compressed)))
    val lengthVec = Wire(Vec(4, chiselTypeOf(zvc_8b.io.length)))

    dataVec(0) := zvc_8b.io.out.bits
    dataVec(1) := zvc_16b.io.out.bits
    dataVec(2) := zvc_32b.io.out.bits
    dataVec(3) := zvc_64b.io.out.bits
    compressedVec(0) := zvc_8b.io.compressed
    compressedVec(1) := zvc_16b.io.compressed
    compressedVec(2) := zvc_32b.io.compressed
    compressedVec(3) := zvc_64b.io.compressed
    lengthVec(0) := Mux(zvc_8b.io.compressed, zvc_8b.io.length, (zvc_8b.io.out.bits.getWidth - 1).U)
    lengthVec(1) := Mux(zvc_16b.io.compressed, zvc_16b.io.length, (zvc_16b.io.out.bits.getWidth - 1).U)
    lengthVec(2) := Mux(zvc_32b.io.compressed, zvc_32b.io.length, (zvc_32b.io.out.bits.getWidth - 1).U)
    lengthVec(3) := Mux(zvc_64b.io.compressed, zvc_64b.io.length, (zvc_64b.io.out.bits.getWidth - 1).U)
    assert(zvc_8b.io.compressed || lengthVec(0) === (zvc_8b.io.out.bits.getWidth - 1).U)

    val lengthMin = ParallelMin(lengthVec)
    val dataId = PriorityEncoder(VecInit(lengthVec.map(_ === lengthMin)))
    val dataSelected = dataVec(dataId)

    val zvcId = dataId
    val header = Cat(zvcId, bitPlaneEnable, dgbId)
    val combined = Cat(header, dataSelected)
    val totalLen = header.getWidth.U +& lengthVec(zvcId)
    val canCompress = compressedVec.asUInt.orR && (totalLen < rawData.getWidth.U)

    val compressedData = Wire(chiselTypeOf(in))
    compressedData.valid := in.valid
    compressedData.bits := Mux(canCompress, combined(combined.getWidth - 1, header.getWidth), rawData)
    assert(canCompress || rawData === compressedData.bits)
    XSPerfAccumulate("ZeroValuePattern", compressedData.valid && canCompress && dgbId === 0.U)
    XSPerfAccumulate("ValueLocality8bPattern", compressedData.valid && canCompress && dgbId === 1.U)
    XSPerfAccumulate("ValueLocality16bPattern", compressedData.valid && canCompress && dgbId === 2.U)
    XSPerfAccumulate("ValueLocality32bPattern", compressedData.valid && canCompress && dgbId === 3.U)
    XSPerfAccumulate("ValueLocality64bPattern", compressedData.valid && canCompress && dgbId === 4.U)
    XSPerfAccumulate("OtherPattern", compressedData.valid && !canCompress)
    (compressedData, canCompress, totalLen)
  }
}
