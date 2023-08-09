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
import freechips.rocketchip.tilelink.TLHints._
import coupledL2.prefetch.PrefetchReq
import coupledL2.utils.XSPerfAccumulate

class SinkA(implicit p: Parameters) extends L2Module {
  val io = IO(new Bundle() {
    val a = Flipped(DecoupledIO(new TLBundleA(edgeIn.bundle)))
    val prefetchReq = prefetchOpt.map(_ => Flipped(DecoupledIO(new PrefetchReq)))
    val toReqArb = DecoupledIO(new TaskBundle)
  })

//  val (first, last, done, count) = edgeIn.count(io.a)
  assert(!(io.a.valid && io.a.bits.opcode(2, 1) === 0.U), "no Put")

  val commonReq = Wire(io.toReqArb.cloneType)
  val prefetchReq = prefetchOpt.map(_ => Wire(io.toReqArb.cloneType))

  io.a.ready := commonReq.ready

  def fromTLAtoTaskBundle(a: TLBundleA): TaskBundle = {
    val task = Wire(new TaskBundle)
    task := DontCare
    task.channel := "b001".U
    task.tag := parseAddress(a.address)._1
    task.set := parseAddress(a.address)._2
    task.off := parseAddress(a.address)._3
    task.alias.foreach(_ := a.user.lift(AliasKey).getOrElse(0.U))
    task.opcode := a.opcode
    task.param := a.param
    task.size := a.size
    task.sourceId := a.source
    task.mshrTask := false.B
    task.fromL2pft.foreach(_ := false.B)
    task.needHint.foreach(_ := a.user.lift(PrefetchKey).getOrElse(false.B))
    task.reqSource := a.user.lift(utility.ReqSourceKey).getOrElse(MemReqSource.NoWhere.id.U)
    task
  }
  def fromPrefetchReqtoTaskBundle(req: PrefetchReq): TaskBundle = {
    val task = Wire(new TaskBundle)
    val fullAddr = Cat(req.tag, req.set, 0.U(offsetBits.W))
    task := DontCare
    task.channel := "b001".U
    task.tag := parseAddress(fullAddr)._1
    task.set := parseAddress(fullAddr)._2
    task.off := 0.U
    task.alias.foreach(_ := 0.U)
    task.opcode := Hint
    task.param := Mux(req.needT, PREFETCH_WRITE, PREFETCH_READ)
    task.size := offsetBits.U
    task.sourceId := req.source
    task.needProbeAckData := false.B
    task.mshrTask := false.B
    task.aliasTask.foreach(_ := false.B)
    task.fromL2pft.foreach(_ := req.isBOP)
    task.needHint.foreach(_ := false.B)
    task.reqSource := MemReqSource.L2Prefetch.id.U
    task
  }
  commonReq.valid := io.a.valid
  commonReq.bits := fromTLAtoTaskBundle(io.a.bits)
  if (prefetchOpt.nonEmpty) {
    prefetchReq.get.valid := io.prefetchReq.get.valid
    prefetchReq.get.bits := fromPrefetchReqtoTaskBundle(io.prefetchReq.get.bits)
    io.prefetchReq.get.ready := prefetchReq.get.ready
    fastArb(Seq(commonReq, prefetchReq.get), io.toReqArb)
  } else {
    io.toReqArb <> commonReq
  }

  // Performance counters
  // num of reqs
  XSPerfAccumulate(cacheParams, "sinkA_req", io.toReqArb.fire())
  XSPerfAccumulate(cacheParams, "sinkA_acquire_req", io.a.fire() && io.a.bits.opcode(2, 1) === AcquireBlock(2, 1))
  XSPerfAccumulate(cacheParams, "sinkA_acquireblock_req", io.a.fire() && io.a.bits.opcode === AcquireBlock)
  XSPerfAccumulate(cacheParams, "sinkA_acquireperm_req", io.a.fire() && io.a.bits.opcode === AcquirePerm)
  XSPerfAccumulate(cacheParams, "sinkA_get_req", io.a.fire() && io.a.bits.opcode === Get)
  prefetchOpt.foreach { _ => XSPerfAccumulate(cacheParams, "sinkA_prefetch_req", io.prefetchReq.get.fire()) }

  // cycels stalled by mainpipe
  val stall = io.toReqArb.valid && !io.toReqArb.ready
  XSPerfAccumulate(cacheParams, "sinkA_stall_by_mainpipe", stall)
  XSPerfAccumulate(cacheParams, "sinkA_acquire_stall_by_mainpipe", stall &&
    (io.toReqArb.bits.opcode === AcquireBlock || io.toReqArb.bits.opcode === AcquirePerm))
  XSPerfAccumulate(cacheParams, "sinkA_get_stall_by_mainpipe", stall && io.toReqArb.bits.opcode === Get)
  prefetchOpt.foreach { _ => XSPerfAccumulate(cacheParams, "sinkA_prefetch_stall_by_mainpipe", stall && io.toReqArb.bits.opcode === Hint) }
}