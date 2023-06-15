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

package coupledL2

import chisel3._
import chisel3.util._
import chipsalliance.rocketchip.config.Parameters
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLMessages._
import coupledL2.utils.XSPerfAccumulate

class SinkB(implicit p: Parameters) extends L2Module {
  val io = IO(new Bundle() {
    val b = Flipped(DecoupledIO(new TLBundleB(edgeIn.bundle)))
    val task = DecoupledIO(new TaskBundle)

    val BTag = Output(UInt(tagBits.W))
    val BSet = Output(UInt(setBits.W))
  })

  def fromTLBtoTaskBundle(b: TLBundleB): TaskBundle = {
    val task = Wire(new TaskBundle)
    task := DontCare
    task.channel := "b010".U
    task.tag := parseAddress(b.address)._1
    task.set := parseAddress(b.address)._2
    task.off := parseAddress(b.address)._3
    task.alias.foreach(_ := 0.U)
    task.opcode := b.opcode
    task.param := b.param
    task.size := b.size
    task.needProbeAckData := b.data(0) // TODO: parameterize this
    task.mshrTask := false.B
    task.fromL2pft.foreach(_ := false.B)
    task.needHint.foreach(_ := false.B)
    task.wayMask := Fill(cacheParams.ways, "b1".U)
    task
  }

  val bTask = fromTLBtoTaskBundle(io.b.bits)
  // grantBuf BlockB logic is complicated, thus having timing issues
  // we have to latch one cycle for fromGrantBuffer.blockSinkReqEntrance.blockB_s1 in ReqArb
  // so we need to give BTag and BSet one cycle ahead of BTask
  val queue = Module(new Queue(new TaskBundle, 1))
  queue.io.enq.valid := io.b.valid
  queue.io.enq.bits := bTask
  io.b.ready := queue.io.enq.ready

  io.task <> queue.io.deq

  // if B req is stalled in Queue, we should use BTag and BSet of the Queue instead of io.b
  io.BTag := Mux(io.b.ready, bTask.tag, queue.io.deq.bits.tag)
  io.BSet := Mux(io.b.ready, bTask.set, queue.io.deq.bits.set)
}