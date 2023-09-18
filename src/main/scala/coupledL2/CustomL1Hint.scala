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
import utility._
import chipsalliance.rocketchip.config.Parameters
import freechips.rocketchip.tilelink.TLMessages._
import coupledL2.utils._

class CustomL1HintIOBundle(implicit p: Parameters) extends L2Bundle {
  // input information
  val s1 = Flipped(ValidIO(new TaskBundle()))
  val s2 = Flipped(ValidIO(new TaskBundle()))
  val s3 = new L2Bundle {
      val task      = Flipped(ValidIO(new TaskBundle()))
      val d         = Input(Bool()) // d_s3.valid
      val need_mshr = Input(Bool())
  }
  val s4 = new L2Bundle {
      val task                  = Flipped(ValidIO(new TaskBundle()))
      val d                     = Input(Bool())
      val need_write_releaseBuf = Input(Bool())
      val need_write_refillBuf  = Input(Bool())
  }
  val s5 = new L2Bundle {
      val task = Flipped(ValidIO(new TaskBundle()))
      val d    = Input(Bool())
  }
  val globalCounter   = Input(UInt((log2Ceil(mshrsAll) + 1).W))
  val grantBufferHint = Flipped(DecoupledIO(new L2ToL1Hint()))

  // output hint
  val l1Hint = DecoupledIO(new L2ToL1Hint())
}


// grantData hint interface
// use this interface to give a hint to l1 before actually sending a GrantData

// TODO: bit extended to UInt
class CustomL1Hint(implicit p: Parameters) extends L2Module {
  val io = IO(new CustomL1HintIOBundle)

  val (task_s1, task_s2, task_s3, task_s4, task_s5) = (io.s1, io.s2, io.s3.task, io.s4.task, io.s5.task)
  val (hasData_s3, hasData_s4, hasData_s5) = (task_s3.bits.opcode(0), task_s4.bits.opcode(0), task_s5.bits.opcode(0))

  // this is d_s3.valid in MainPipe
  val d_s3 = io.s3.d
  val d_s4 = io.s4.d
  val d_s5 = io.s5.d

  require(hintCycleAhead <= 3)

  // only use lower 2 bits of io.globalCounter to make timing happy
  // as main pipeline will not trigger hint if io.globalCounter >= hintCycleAhead
  val globalCounter   = io.globalCounter // TODO??WARNING: WHY 2 bit??
  val grantBufferHint = io.grantBufferHint

  val hint_s1, hint_s2, hint_s3, hint_s4, hint_s5 = Wire(Valid(new L2ToL1Hint()))

  val (mshrReq_s1, mshrReq_s2, mshrReq_s3, mshrReq_s4) =
    (task_s1.bits.mshrTask, task_s2.bits.mshrTask, task_s3.bits.mshrTask, task_s4.bits.mshrTask)

  val need_mshr_s3 = io.s3.need_mshr

  // =========== S5 Hint ===========
  //HINTWARNING: should this be Grant/GrantData?
  // task@s5 is sure to enter GrantBuf the next cycle, so it will fire at D after (1 + globalCounter) cycles
  hint_s5.valid         := d_s5 && task_s5.bits.fromA && hasData_s5 && ((globalCounter + 1.U) === hintCycleAhead.U)
  hint_s5.bits.sourceId := task_s5.bits.sourceId

  // =========== S4 Hint ===========
 /** task-GrantData@s4 is divided into 2 situations (mshrTask or chnTask), and this will affect when to Hint
  * if !mshrTask, it is a response to an A-channel-hit req, and will not be sent @s4, but @s5
  * if  mshrTask, it is a MSHR task, and is ok to be sent @s4
  */
  val chn_s4  = task_s4.valid && !mshrReq_s4 && task_s4.bits.fromA && task_s4.bits.opcode === GrantData
  val mshr_s4 =          d_s4 &&  mshrReq_s4 && task_s4.bits.fromA && task_s4.bits.opcode === GrantData

  /** For ChnTask@s4
   * it must wait until @s5 to enter GrantBuf, so it will fire at D after (2 + globalCounter) cycles;
   * if s5 has data, an extra cycle is needed, so it will fire at D after (3 + globalCounter) cycles;
   */
  val chnCntMatch_s4 = Mux(d_s5 && hasData_s5, (globalCounter + 3.U) === hintCycleAhead.U,
    (globalCounter + 2.U) === hintCycleAhead.U)

  /** For MSHRTask@s4
  * if s5 has no task, task@s4 is sure to enter GrantBuf the next cycle, same as above;
  * if s5 has task, s5 will enter GrantBuf first, then s4, so it will fire at D after (2 + globalCounter) cycles;
  * if s5 has data, an extra cycle is needed, so it will fire at D after (3 + globalCounter) cycles;
  */
  val mshrCntMatch_s4 = Mux(!d_s5, (globalCounter + 1.U) === hintCycleAhead.U,
    Mux(!hasData_s5, (globalCounter + 2.U) === hintCycleAhead.U, (globalCounter + 3.U) === hintCycleAhead.U))

  hint_s4.valid := (chn_s4 && chnCntMatch_s4) || (mshr_s4 && mshrCntMatch_s4)
  hint_s4.bits.sourceId := task_s4.bits.sourceId

  // TODO: EXCEPTION detected
  //  now s4 & s5 is empty, s3 is chnHit, s2 is mshrTask, we suppose chnHit@s3 should Hint
  //  the next cycle, s4 is chnHit, s3 is mshrTask, and mshrTask@s3 will enter GrantBuf prior to chnHit
  //  so Hint for chnTask is wrong (under current logic, chnTask will also Hint @s5, double Hint)
  //  (Hope this is rare

  // =========== S3 Hint ===========
  val chn_s3  = task_s3.valid && !mshrReq_s3 && !need_mshr_s3 && task_s3.bits.fromA && task_s3.bits.opcode === AcquireBlock
  val mshr_s3 =          d_s3 &&  mshrReq_s3 &&                  task_s3.bits.fromA && task_s3.bits.opcode === GrantData

  /** For ChnTask@s3
   * it must wait until @s5 to enter GrantBuf, so it will fire at D after (3 + globalCounter) cycles;
   * any data @s4/s5 will add an additional cycle, chnTaskHit@s4 will add an extra cycle
   */
  val chnHit_s4 = !d_s4 && task_s4.valid && !mshrReq_s4 && task_s4.bits.fromA
  val chnCntMatch_s3 = (globalCounter + 3.U + (d_s5 & hasData_s5).asUInt + (d_s4 & hasData_s4).asUInt +
    (chnHit_s4 & hasData_s4).asUInt) === hintCycleAhead.U

  /** For MSHRTask@s3
   * if s4/s5 has no task, task@s3 will enter GrantBuf the next cycle, so firing at D after (1 + globalCounter) cycles;
   * any task @s4/s5 will add an additional cycle, taskHasData @s4/s5 will add an extra cycle;
   * task @s5 and chnTaskHit@s4 will add an another extra cycle
   */
  val mshrCntMatch_s3 = (globalCounter + 1.U + d_s5.asUInt + d_s4.asUInt +
    (d_s5 & hasData_s5).asUInt + (d_s4 & hasData_s4).asUInt +
    (d_s5 & (chnHit_s4 + chnHit_s4 & hasData_s4)).asUInt
    ) === hintCycleAhead.U

  hint_s3.valid := (chn_s3 && chnCntMatch_s3) || (mshr_s3 && mshrCntMatch_s3)
  hint_s3.bits.sourceId := task_s3.bits.sourceId

  // =========== S2 Hint ===========
  /** task-GrantData@s2 is only valid for mshrTask
   */
  val mshr_s2 = task_s2.valid && mshrReq_s2 && task_s2.bits.fromA && task_s2.bits.opcode === GrantData

  /** For MSHRTask@s2
   * Basics are the same as above (s3/4/5 has task, and has data)
   * The following may be too complicated to make it 100% correct
   */
   val chnHit_s3 = !d_s3 && task_s3.valid && !mshrReq_s3 && task_s3.bits.fromA
  val mshrCntMatch_s2 = (globalCounter + 2.U + d_s3.asUInt + d_s4.asUInt + d_s5.asUInt +
    (d_s3 & hasData_s3).asUInt + (d_s4 & hasData_s4).asUInt + (d_s5 & hasData_s5).asUInt +
    (chnHit_s4 + chnHit_s4 & hasData_s4) +
    d_s4 & d_s5 & (chnHit_s3 + chnHit_s3 & hasData_s3)
    ) === hintCycleAhead.U

  hint_s2.valid := mshr_s2 && mshrCntMatch_s2
  hint_s2.bits.sourceId := task_s2.bits.sourceId

  // =========== S1 Hint ===========
  /** task-GrantData@s1 is only valid for mshrTask
   * mshrTask@s1 already takes 3 cycles to fire, Only when all clear can s1 send Hint
   */
  val mshr_s1 = task_s1.valid && mshrReq_s1 && task_s1.bits.fromA && task_s1.bits.opcode === GrantData
  val hasReq_s2 = task_s2.valid && task_s2.bits.fromA // this is kind of speculative, cuz we do not know it will hit or not
  val mshrCntMatch_s1 = Cat(d_s5, d_s4, chnHit_s4, d_s3, chnHit_s3, hasReq_s2, globalCounter) === 0.U

  hint_s1.valid := mshr_s1 && mshrCntMatch_s1
  hint_s1.bits.sourceId := task_s1.bits.sourceId

  // =========== Arb all hints ===========
  // saves valid hint_s12345 when !io.hint.ready
  // HINTWARNING: we may quit setting flow=true, and move the latch1 that is previously at CoupledL2-L1 to the hintqueue here
  val mpHintQueue = Module(new Queue(new L2ToL1Hint(), entries = 8, flow = true)) // HINTWARNING: assert queue never overflow (enq !ready & valid)

  val mpHints = Seq(hint_s5, hint_s4, hint_s3, hint_s2, hint_s1)
  val mpHintValids = mpHints.map(_.valid)
  val mpHintSourceIds = mpHints.map(_.bits.sourceId)

  mpHintQueue.io.enq.valid := VecInit(mpHintValids).asUInt.orR
  mpHintQueue.io.enq.bits.sourceId := ParallelPriorityMux(mpHintValids zip mpHintSourceIds)
  mpHintQueue.io.deq.ready := io.l1Hint.ready && !grantBufferHint.valid
  grantBufferHint.ready := io.l1Hint.ready

  // final decision
  io.l1Hint.valid         := grantBufferHint.valid || mpHintQueue.io.deq.valid
  io.l1Hint.bits.sourceId := Mux(grantBufferHint.valid, grantBufferHint.bits.sourceId, mpHintQueue.io.deq.bits.sourceId)

  // TODO: open this assert when hint is TRULY accurate for all situations
//  assert(PopCount(Cat(VecInit(mpHintValids).asUInt, grantBufferHint.valid)) <= 1.U)

  val impossible_pipe_hint = globalCounter > hintCycleAhead.U
  assert(!(impossible_pipe_hint & mpHintQueue.io.enq.valid), "Hint should not come from MainPipe")

  XSPerfAccumulate(cacheParams, "hint_grantBufferHint_valid", grantBufferHint.valid)
  XSPerfAccumulate(cacheParams, "hint_s1_valid", hint_s1.valid)
  XSPerfAccumulate(cacheParams, "hint_s2_valid", hint_s2.valid)
  XSPerfAccumulate(cacheParams, "hint_s3_valid", hint_s3.valid)
  XSPerfAccumulate(cacheParams, "hint_s4_valid", hint_s4.valid)
  XSPerfAccumulate(cacheParams, "hint_s5_valid", hint_s5.valid)
  XSPerfAccumulate(cacheParams, "incorrect_hint", PopCount(VecInit(mpHintValids)) > 1.U)

}