// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket

import Chisel._
import Chisel.ImplicitConversions._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.subsystem.RocketTilesKey
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import chisel3.internal.sourceinfo.SourceInfo
import chisel3.experimental.dontTouch
import chisel3.experimental.chiselName

case class ICacheParams(
    nSets: Int = 64,
    nWays: Int = 4,
    rowBits: Int = 128,
    nTLBEntries: Int = 32,
    cacheIdBits: Int = 0,
    tagECC: Option[String] = None,
    dataECC: Option[String] = None,
    itimAddr: Option[BigInt] = None,
    prefetch: Boolean = false,
    blockBytes: Int = 64,
    latency: Int = 2,
    fetchBytes: Int = 4) extends L1CacheParams {
  def tagCode: Code = Code.fromString(tagECC)
  def dataCode: Code = Code.fromString(dataECC)
  def replacement = new RandomReplacement(nWays)
}

trait HasL1ICacheParameters extends HasL1CacheParameters with HasCoreParameters {
  val cacheParams = tileParams.icache.get
}

class ICacheReq(implicit p: Parameters) extends CoreBundle()(p) with HasL1ICacheParameters {
  val addr = UInt(width = vaddrBits)
}

class ICacheErrors(implicit p: Parameters) extends CoreBundle()(p)
    with HasL1ICacheParameters
    with CanHaveErrors {
  val correctable = (cacheParams.tagCode.canDetect || cacheParams.dataCode.canDetect).option(Valid(UInt(width = paddrBits)))
  val uncorrectable = (cacheParams.itimAddr.nonEmpty && cacheParams.dataCode.canDetect).option(Valid(UInt(width = paddrBits)))
}

class ICache(val icacheParams: ICacheParams, val hartId: Int)(implicit p: Parameters) extends LazyModule {
  lazy val module = new ICacheModule(this)
  val masterNode = TLClientNode(Seq(TLClientPortParameters(Seq(TLClientParameters(
    sourceId = IdRange(0, 1 + icacheParams.prefetch.toInt), // 0=refill, 1=hint
    name = s"Core ${hartId} ICache")))))

  val size = icacheParams.nSets * icacheParams.nWays * icacheParams.blockBytes
  val device = new SimpleDevice("itim", Seq("sifive,itim0"))
  private val wordBytes = icacheParams.fetchBytes
  val slaveNode =
    TLManagerNode(icacheParams.itimAddr.toSeq.map { itimAddr => TLManagerPortParameters(
      Seq(TLManagerParameters(
        address         = Seq(AddressSet(itimAddr, size-1)),
        resources       = device.reg("mem"),
        regionType      = RegionType.UNCACHEABLE,
        executable      = true,
        supportsPutFull = TransferSizes(1, wordBytes),
        supportsPutPartial = TransferSizes(1, wordBytes),
        supportsGet     = TransferSizes(1, wordBytes),
        fifoId          = Some(0))), // requests handled in FIFO order
      beatBytes = wordBytes,
      minLatency = 1)})
}

class ICacheResp(outer: ICache) extends Bundle {
  val data = UInt(width = outer.icacheParams.fetchBytes*8)
  val replay = Bool()
  val ae = Bool()

  override def cloneType = new ICacheResp(outer).asInstanceOf[this.type]
}

class ICachePerfEvents extends Bundle {
  val acquire = Bool()
}

class ICacheBundle(val outer: ICache) extends CoreBundle()(outer.p) {
  val hartid = UInt(INPUT, hartIdLen)
  val req = Decoupled(new ICacheReq).flip
  val s1_paddr = UInt(INPUT, paddrBits) // delayed one cycle w.r.t. req
  val s2_vaddr = UInt(INPUT, vaddrBits) // delayed two cycles w.r.t. req
  val s1_kill = Bool(INPUT) // delayed one cycle w.r.t. req
  val s2_kill = Bool(INPUT) // delayed two cycles; prevents I$ miss emission
  val s2_prefetch = Bool(INPUT) // should I$ prefetch next line on a miss?

  val resp = Valid(new ICacheResp(outer))
  val invalidate = Bool(INPUT)

  val errors = new ICacheErrors
  val perf = new ICachePerfEvents().asOutput
}

// get a tile-specific property without breaking deduplication
object GetPropertyByHartId {
  def apply[T <: Data](tiles: Seq[RocketTileParams], f: RocketTileParams => Option[T], hartId: UInt): T = {
    PriorityMux(tiles.collect { case t if f(t).isDefined => (t.hartId === hartId) -> f(t).get })
  }
}
@chiselName
class ICacheModule(outer: ICache) extends LazyModuleImp(outer)
    with HasL1ICacheParameters {
  override val cacheParams = outer.icacheParams // Use the local parameters

  val io = IO(new ICacheBundle(outer))
  val (tl_out, edge_out) = outer.masterNode.out(0)
  // Option.unzip does not exist :-(
  //val (tl_in, edge_in) = outer.slaveNode.in.headOption.unzip //tl_in is disabled

  val tECC = cacheParams.tagCode
  val dECC = cacheParams.dataCode

  require(isPow2(nSets) && isPow2(nWays))
  require(!usingVM || pgIdxBits >= untagBits)

  val s1_valid = Reg(init=Bool(false))
  val s1_tag_hit = Wire(Vec(nWays, Bool()))
  val s1_hit = s1_tag_hit.reduce(_||_)
  dontTouch(s1_hit)
  val s2_valid = RegNext(s1_valid && !io.s1_kill, Bool(false))
  val s2_hit = RegNext(s1_hit)

  val invalidated = Reg(Bool())
  val refill_valid = RegInit(false.B)
  val refill_fire = tl_out.a.fire()
  val s2_miss = s2_valid && !s2_hit && !io.s2_kill && !RegNext(refill_valid)
  val refill_addr = RegEnable(io.s1_paddr, s1_valid && !(refill_valid || s2_miss))
  val refill_tag = refill_addr(tagBits+untagBits-1,untagBits)
  val refill_idx = refill_addr(untagBits-1,blockOffBits)
  val refill_one_beat = tl_out.d.fire() && edge_out.hasData(tl_out.d.bits)

  io.req.ready := !(refill_one_beat)
  val s0_valid = io.req.fire()
  val s0_vaddr = io.req.bits.addr
  s1_valid := s0_valid
  val s_1_vaddr = dontTouch(RegNext(s0_vaddr))

  val refill_vaddr = dontTouch(RegEnable(s_1_vaddr, s1_valid && !(refill_valid || s2_miss))) //New
  val refill_vtag = refill_vaddr(tagBits+untagBits-1,untagBits) //New
  val refill_vidx = refill_vaddr(untagBits-1,blockOffBits) //New

  val (_, _, d_done, refill_cnt) = edge_out.count(tl_out.d)
  val refill_done = refill_one_beat && d_done
  tl_out.d.ready := !false
  require (edge_out.manager.minLatency > 0)


  val repl_way = if (isDM) UInt(0) else {
    // pick a way that is not used by the scratchpad
    val v0 = LFSR16(refill_fire)(log2Up(nWays)-1,0)
    var v = v0
    v
  }

  val vb_array = Reg(init=Bits(0, nSets*nWays))
  when (refill_one_beat) {
    // clear bit when refill starts so hit-under-miss doesn't fetch bad data
    vb_array := vb_array.bitSet(Cat(repl_way, refill_idx), refill_done && !invalidated)
    
  }

  val tag_ways = dontTouch(Reg(Vec(nSets*nWays, UInt(width = 8))))
  val s0_way_idx = s0_vaddr(untagBits-1,blockOffBits)
  val s0_way_tag = s0_vaddr(tagBits+untagBits-1,untagBits)
  val s0_tag_index = Cat(s0_way_tag, s0_way_idx)
  val s0_way_tag_xor = s0_tag_index(7,0) ^ s0_tag_index(15,8) ^ s0_tag_index(23,16)  
 
  val vb_way0 = vb_array(Cat(UInt(0), s0_way_idx))
  val vb_way1 = vb_array(Cat(UInt(1), s0_way_idx))
  val vb_way2 = vb_array(Cat(UInt(2), s0_way_idx))
  val vb_way3 = vb_array(Cat(UInt(3), s0_way_idx))  
 
  val way_0 = (tag_ways(Cat(UInt(0), s0_way_idx)) === s0_way_tag_xor && vb_way0)
  val way_1 = (tag_ways(Cat(UInt(1), s0_way_idx)) === s0_way_tag_xor && vb_way1)
  val way_2 = (tag_ways(Cat(UInt(2), s0_way_idx)) === s0_way_tag_xor && vb_way2)
  val way_3 = (tag_ways(Cat(UInt(3), s0_way_idx)) === s0_way_tag_xor && vb_way3)

  val way_seq = Seq(way_0, way_1, way_2, way_3)
  //val way_seq = dontTouch(Reg(Vec(nWays, Bool())))
  //for (i <- 0 until nWays) {
  //  way_seq(i) := vb_array(Cat(UInt(i), s1_way_idx)) && (tag_ways(Cat(UInt(i), s1_way_idx)) === s1_way_tag)
  //}
  val way_not_detected = (PopCount(way_seq) === 0.U)

  val cntRegWay0 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay1 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay2 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay3 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay4 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay5 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay6 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay7 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay8 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay9 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay10 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay11 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay12 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay13 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay14 = dontTouch(RegInit (0.U(32.W))) //New
  val cntRegWay15 = dontTouch(RegInit (0.U(32.W))) //New

  
  
  when( !refill_done && s0_valid && !way_0 && !way_1 && !way_2 && !way_3 && !refill_valid) {
    cntRegWay0 := cntRegWay0 + 1.U
  }
  when( !refill_done && s0_valid && way_0 && !way_1 && !way_2 && !way_3 && !refill_valid) {
    cntRegWay1 := cntRegWay1 + 1.U
  }
  when( !refill_done && s0_valid && !way_0 && way_1 && !way_2 && !way_3 && !refill_valid) {
    cntRegWay2 := cntRegWay2 + 1.U
  }
  when( !refill_done && s0_valid && way_0 && way_1 && !way_2 && !way_3 && !refill_valid) {
    cntRegWay3 := cntRegWay3 + 1.U
  }
  when( !refill_done && s0_valid && !way_0 && !way_1 && way_2 && !way_3 && !refill_valid) {
    cntRegWay4 := cntRegWay4 + 1.U
  }
  when( !refill_done && s0_valid && way_0 && !way_1 && way_2 && !way_3 && !refill_valid) {
    cntRegWay5 := cntRegWay5 + 1.U
  }
  when( !refill_done && s0_valid && !way_0 && way_1 && way_2 && !way_3 && !refill_valid) {
    cntRegWay6 := cntRegWay6 + 1.U
  }
  when( !refill_done && s0_valid && way_0 && way_1 && way_2 && !way_3 && !refill_valid) {
    cntRegWay7 := cntRegWay7 + 1.U
  }
  when( !refill_done && s0_valid && !way_0 && !way_1 && !way_2 && way_3 && !refill_valid) {
    cntRegWay8 := cntRegWay8 + 1.U
  }
  when( !refill_done && s0_valid && way_0 && !way_1 && !way_2 && way_3 && !refill_valid) {
    cntRegWay9 := cntRegWay9 + 1.U
  }
  when( !refill_done && s0_valid && !way_0 && way_1 && !way_2 && way_3 && !refill_valid) {
    cntRegWay10 := cntRegWay10 + 1.U
  }
  when( !refill_done && s0_valid && way_0 && way_1 && !way_2 && way_3 && !refill_valid) {
    cntRegWay11 := cntRegWay11 + 1.U
  }
  when( !refill_done && s0_valid && !way_0 && !way_1 && way_2 && way_3 && !refill_valid) {
    cntRegWay12 := cntRegWay12 + 1.U
  }
  when( !refill_done && s0_valid && way_0 && !way_1 && way_2 && way_3 && !refill_valid) {
    cntRegWay13 := cntRegWay13 + 1.U
  }
  when( !refill_done && s0_valid && !way_0 && way_1 && way_2 && way_3 && !refill_valid) {
    cntRegWay14 := cntRegWay14 + 1.U
  }
  when( !refill_done && s0_valid && way_0 && way_1 && way_2 && way_3 && !refill_valid) {
    cntRegWay15 := cntRegWay15 + 1.U
  }

  // 4 SRAMs, 1 SRAM for each way
  //val tag_arrays = Seq.fill(nWays)(SeqMem(nSets, UInt(width = tECC.width(1 + tagBits))))
  val tag_array_0 = SeqMem(nSets, UInt(width = tECC.width(1 + tagBits)))
  val tag_array_1 = SeqMem(nSets, UInt(width = tECC.width(1 + tagBits)))
  val tag_array_2 = SeqMem(nSets, UInt(width = tECC.width(1 + tagBits))) //4
  val tag_array_3 = SeqMem(nSets, UInt(width = tECC.width(1 + tagBits))) //4
  // Read all the ways in parallel
  val tag_0 = tag_array_0.read(s0_vaddr(untagBits-1,blockOffBits), !refill_done && s0_valid && (way_seq(0) || way_not_detected))
  val tag_1 = tag_array_1.read(s0_vaddr(untagBits-1,blockOffBits), !refill_done && s0_valid && (way_seq(1) || way_not_detected))
  val tag_2 = tag_array_2.read(s0_vaddr(untagBits-1,blockOffBits), !refill_done && s0_valid && (way_seq(2) || way_not_detected)) //4
  val tag_3 = tag_array_3.read(s0_vaddr(untagBits-1,blockOffBits), !refill_done && s0_valid && (way_seq(3) || way_not_detected)) //4

  val tag_rdata = Vec(tag_0, tag_1, tag_2, tag_3) //4 Vec(tag_0, tag_1) //2 
  
  val accruedRefillError = Reg(Bool())
  
  when (refill_done) {
    //for ((tag_array, i) <- tag_arrays zipWithIndex) {
      //when(repl_way === i.U) {
        val enc_tag = tECC.encode(Cat(tl_out.d.bits.error, refill_tag))

        when (repl_way === 0.U) {tag_array_0.write(refill_idx, enc_tag)}
        .elsewhen (repl_way === 1.U) {tag_array_1.write(refill_idx, enc_tag)} //4 
        .elsewhen (repl_way === 2.U) {tag_array_2.write(refill_idx, enc_tag)} //4
        .otherwise {tag_array_3.write(refill_idx, enc_tag)} //4

	val tag_index = Cat(refill_vtag, refill_vidx)
        tag_ways(Cat(repl_way, refill_idx)) := tag_index(7,0) ^ tag_index(15,8) ^ tag_index(23,16)
        ccover(tl_out.d.bits.error, "D_ERROR", "I$ D-channel error") 
      //}
    //}
  }

  
  val invalidate = dontTouch(Wire(init = io.invalidate))
  when (invalidate) {
    vb_array := Bits(0)
    invalidated := Bool(true)
  }

  val s1_tag_disparity = dontTouch(Wire(Vec(nWays, Bool())))
  val s1_tl_error = Wire(Vec(nWays, Bool()))
  val wordBits = outer.icacheParams.fetchBytes*8

  val s1_dout_0 = dontTouch(Wire(UInt(width = dECC.width(wordBits))))
  val s1_dout_1 = dontTouch(Wire(UInt(width = dECC.width(wordBits))))
  val s1_dout_2 = dontTouch(Wire(UInt(width = dECC.width(wordBits)))) //4
  val s1_dout_3 = dontTouch(Wire(UInt(width = dECC.width(wordBits)))) //4
  
  //val s1_dout = Wire(Vec(nWays, UInt(width = dECC.width(wordBits))))
  val s1_dout = Wire(init=Vec(s1_dout_0, s1_dout_1, s1_dout_2, s1_dout_3)) //4 Wire(init=Vec(s1_dout_0, s1_dout_1)) //2 

  val cntRegData = dontTouch(RegInit (0.U(32.W))) //New
  
  for (i <- 0 until nWays) {
    val s1_idx = io.s1_paddr(untagBits-1,blockOffBits)
    val s1_tag = io.s1_paddr(tagBits+untagBits-1,untagBits)
    val s1_vb = vb_array(Cat(UInt(i), s1_idx))
    val enc_tag = tECC.decode(tag_rdata(i))
    val (tl_error, tag) = Split(enc_tag.uncorrected, tagBits)
    val tagMatch = s1_vb && tag === s1_tag
    s1_tag_disparity(i) := s1_vb && enc_tag.error
    s1_tl_error(i) := tagMatch && tl_error.toBool
    s1_tag_hit(i) := tagMatch
  }
  assert(PopCount(s1_tag_hit zip s1_tag_disparity map { case (h, d) => h && !d }) <= 1)

  require(tl_out.d.bits.data.getWidth % wordBits == 0)
  val data_arrays_0 = Seq.fill(tl_out.d.bits.data.getWidth/wordBits) {SeqMem(nSets*refillCycles, UInt(width = dECC.width(wordBits)))}
  val data_arrays_1 = Seq.fill(tl_out.d.bits.data.getWidth/wordBits) {SeqMem(nSets*refillCycles, UInt(width = dECC.width(wordBits)))}
  val data_arrays_2 = Seq.fill(tl_out.d.bits.data.getWidth/wordBits) {SeqMem(nSets*refillCycles, UInt(width = dECC.width(wordBits)))} //4
  val data_arrays_3 = Seq.fill(tl_out.d.bits.data.getWidth/wordBits) {SeqMem(nSets*refillCycles, UInt(width = dECC.width(wordBits)))} //4

  for ((data_array, i) <- data_arrays_0 zipWithIndex) {
    def wordMatch(addr: UInt) = addr.extract(log2Ceil(tl_out.d.bits.data.getWidth/8)-1, log2Ceil(wordBits/8)) === i
    def row(addr: UInt) = addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
    val s0_ren = (s0_valid && wordMatch(s0_vaddr))
    val wen = (refill_one_beat && !invalidated)
    val mem_idx = Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt, row(s0_vaddr))
    when (wen) {
      val data = tl_out.d.bits.data(wordBits*(i+1)-1, wordBits*i)
      val way = repl_way
      //for ((data_array_inner, i) <- data_array zipWithIndex) {
        when (way === 0.U) {data_array.write(mem_idx, dECC.encode(data))}
      //}
    }

    //val dout = Reg(Vec(nWays, UInt(width = tECC.width(wordBits)))) //Register for 4 way tags
    when(s0_ren) {
      cntRegData := cntRegData + 1.U
    }

    val dout = data_array.read(mem_idx, !wen && s0_ren && (way_seq(0) || way_not_detected))
    
    //val dout = Vec(data_0, data_1, data_2, data_3)

    when (wordMatch(io.s1_paddr)) {
      //s1_dout(0) := dout
      s1_dout_0 := dout
      //s1_dout_orig(0) := dout
    }
  }

  for ((data_array, i) <- data_arrays_1 zipWithIndex) {
    def wordMatch(addr: UInt) = addr.extract(log2Ceil(tl_out.d.bits.data.getWidth/8)-1, log2Ceil(wordBits/8)) === i
    def row(addr: UInt) = addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
    val s0_ren = (s0_valid && wordMatch(s0_vaddr))
    val wen = (refill_one_beat && !invalidated)
    val mem_idx = Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt, row(s0_vaddr))
    when (wen) {
      val data = tl_out.d.bits.data(wordBits*(i+1)-1, wordBits*i)
      val way = repl_way
      //for ((data_array_inner, i) <- data_array zipWithIndex) {
        when (way === 1.U) {data_array.write(mem_idx, dECC.encode(data))}
      //}
    }

    //val dout = Reg(Vec(nWays, UInt(width = tECC.width(wordBits)))) //Register for 4 way tags
    when(s0_ren) {
      cntRegData := cntRegData + 1.U
    }

    val dout = data_array.read(mem_idx, !wen && s0_ren && (way_seq(1) || way_not_detected))
    
    //val dout = Vec(data_0, data_1, data_2, data_3)

    when (wordMatch(io.s1_paddr)) {
      //s1_dout(1) := dout
      s1_dout_1 := dout
      //s1_dout_orig(1) := dout
    }
  }
 //4
  for ((data_array, i) <- data_arrays_2 zipWithIndex) {
    def wordMatch(addr: UInt) = addr.extract(log2Ceil(tl_out.d.bits.data.getWidth/8)-1, log2Ceil(wordBits/8)) === i
    def row(addr: UInt) = addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
    val s0_ren = (s0_valid && wordMatch(s0_vaddr))
    val wen = (refill_one_beat && !invalidated)
    val mem_idx = Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt, row(s0_vaddr))
    when (wen) {
      val data = tl_out.d.bits.data(wordBits*(i+1)-1, wordBits*i)
      val way = repl_way
      //for ((data_array_inner, i) <- data_array zipWithIndex) {
        when (way === 2.U) {data_array.write(mem_idx, dECC.encode(data))}
      //}
    }
    //val dout = Reg(Vec(nWays, UInt(width = tECC.width(wordBits)))) //Register for 4 way tags
    
    val dout = data_array.read(mem_idx, !wen && s0_ren && (way_seq(2) || way_not_detected))
    
    //val dout = Vec(data_0, data_1, data_2, data_3)
    when (wordMatch(io.s1_paddr)) {
      //s1_dout(2) := dout
      s1_dout_2 := dout
      //s1_dout_orig(2) := dout
    }
  }
  for ((data_array, i) <- data_arrays_3 zipWithIndex) {
    def wordMatch(addr: UInt) = addr.extract(log2Ceil(tl_out.d.bits.data.getWidth/8)-1, log2Ceil(wordBits/8)) === i
    def row(addr: UInt) = addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
    val s0_ren = (s0_valid && wordMatch(s0_vaddr))
    val wen = (refill_one_beat && !invalidated)
    val mem_idx = Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt, row(s0_vaddr))
    when (wen) {
      val data = tl_out.d.bits.data(wordBits*(i+1)-1, wordBits*i)
      val way = repl_way
      //for ((data_array_inner, i) <- data_array zipWithIndex) {
        when (way === 3.U) {data_array.write(mem_idx, dECC.encode(data))}
      //}
    }
    //val dout = Reg(Vec(nWays, UInt(width = tECC.width(wordBits)))) //Register for 4 way tags
    
    val dout = data_array.read(mem_idx, !wen && s0_ren && (way_seq(3) || way_not_detected))
    
    //val dout = Vec(data_0, data_1, data_2, data_3)
    when (wordMatch(io.s1_paddr)) {
      //s1_dout(3) := dout
      s1_dout_3 := dout
      //s1_dout_orig(3) := dout
    }
  }  //4


  val s1_clk_en = s1_valid
  val s2_tag_hit = RegEnable(s1_tag_hit, s1_clk_en)
  val s2_hit_way = OHToUInt(s2_tag_hit)
  val s2_dout = RegEnable(s1_dout, s1_clk_en)
  val s2_way_mux = Mux1H(s2_tag_hit, s2_dout)

  val s2_tag_disparity = RegEnable(s1_tag_disparity, s1_clk_en).asUInt.orR
  val s2_tl_error = RegEnable(s1_tl_error.asUInt.orR, s1_clk_en)
  val s2_data_decoded = dECC.decode(s2_way_mux)
  val s2_disparity = s2_tag_disparity || s2_data_decoded.error

  val cntRegTag = dontTouch(RegInit (0.U(32.W))) //New
  when( !refill_done && s0_valid) {
    cntRegTag := cntRegTag + 1.U
  }

  val cntRegMiss = dontTouch(RegInit (0.U(32.W))) //New
  when( s2_miss) {
    cntRegMiss := cntRegMiss + 1.U
  }

  val cntRegHit = dontTouch(RegInit (0.U(32.W))) //New
  when( s1_hit) {
    cntRegHit := cntRegHit + 1.U
  }
  
  // output signals
  outer.icacheParams.latency match {
    case 1 =>
      require(tECC.isInstanceOf[IdentityCode])
      require(dECC.isInstanceOf[IdentityCode])
      require(outer.icacheParams.itimAddr.isEmpty)
      io.resp.bits.data := Mux1H(s1_tag_hit, s1_dout)
      io.resp.bits.ae := s1_tl_error.asUInt.orR
      io.resp.valid := s1_valid && s1_hit

    case 2 =>
      // when some sort of memory bit error have occurred
      when (s2_valid && s2_disparity) { 
        invalidate := true
        
      }

      io.resp.bits.data := s2_data_decoded.uncorrected
      io.resp.bits.ae := s2_tl_error
      io.resp.bits.replay := s2_disparity
      io.resp.valid := s2_valid && s2_hit

  }

  tl_out.a.valid := s2_miss && !refill_valid
  tl_out.a.bits := edge_out.Get(
                    fromSource = UInt(0),
                    toAddress = (refill_addr >> blockOffBits) << blockOffBits,
                    lgSize = lgCacheBlockBytes)._2
  
  tl_out.b.ready := Bool(true)
  tl_out.c.valid := Bool(false)
  tl_out.e.valid := Bool(false)
  
  when (!refill_valid) { invalidated := false.B }
  when (refill_fire) { refill_valid := true.B }
  when (refill_done) { refill_valid := false.B}

  io.perf.acquire := refill_fire

  ccover((tl_out.a.valid && !tl_out.a.ready), "MISS_A_STALL", "I$ miss blocked by A-channel")
  ccover(invalidate && refill_valid, "FLUSH_DURING_MISS", "I$ flushed during miss")

  def ccover(cond: Bool, label: String, desc: String)(implicit sourceInfo: SourceInfo) =
    cover(cond, s"ICACHE_$label", "MemorySystem;;" + desc)

  val mem_active_valid = Seq(CoverBoolean(s2_valid, Seq("mem_active")))
  val data_error = Seq(
    CoverBoolean(!s2_data_decoded.correctable && !s2_data_decoded.uncorrectable, Seq("no_data_error")),
    CoverBoolean(s2_data_decoded.correctable, Seq("data_correctable_error")),
    CoverBoolean(s2_data_decoded.uncorrectable, Seq("data_uncorrectable_error")))
  val request_source = Seq(
    CoverBoolean(!false, Seq("from_CPU")),
    CoverBoolean(false, Seq("from_TL"))
  )
  val tag_error = Seq(
    CoverBoolean(!s2_tag_disparity, Seq("no_tag_error")),
    CoverBoolean(s2_tag_disparity, Seq("tag_error"))
  )
  val mem_mode = Seq(
    CoverBoolean(false, Seq("ITIM_mode")),
    CoverBoolean(!false, Seq("cache_mode"))
  )

  val error_cross_covers = new CrossProperty(
    Seq(mem_active_valid, data_error, tag_error, request_source, mem_mode),
    Seq(
      // tag error cannot occur in ITIM mode
      Seq("tag_error", "ITIM_mode"),
      // Can only respond to TL in ITIM mode
      Seq("from_TL", "cache_mode")
    ),
    "MemorySystem;;Memory Bit Flip Cross Covers")

  cover(error_cross_covers)
}
