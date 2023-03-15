import FIFO::*;
import FIFOF::*;
import SpecialFIFOs::*;

import RFile::*;
import Defines::*;
import Decode::*;
import Execute::*;

import Scoreboard::*;

typedef struct {
	Word pc;
	Word predicted_pc;
	Bool epoch;
} F2D deriving(Bits, Eq);

typedef struct {
	Word pc;
	Word predicted_pc;
	Bool epoch;
	DecodedInst dInst; 
	Word rVal1; 
	Word rVal2;
} D2E deriving(Bits, Eq);

typedef struct {
	Word pc;
	RIndx dst;

	Bool isMem;

	Word data;
	Bool extendSigned;
	SizeType size;
} E2M deriving(Bits,Eq);

typedef struct {
	RIndx dst;
	Word data;
} BypassTarget deriving(Bits, Eq);

interface ProcessorIfc;
	method ActionValue#(MemReq32) iMemReq;
	method Action iMemResp(Word data);
	method ActionValue#(MemReq32) dMemReq;
	method Action dMemResp(Word data);
endinterface

(* synthesize *)
module mkProcessor(ProcessorIfc);
	Reg#(Word)  pc <- mkReg(0);
	RFile2R1W   rf <- mkRFile2R1W;

	//Reg#(ProcStage) stage <- mkReg(Fetch);

	FIFOF#(F2D) f2d <- mkSizedFIFOF(2);
        FIFOF#(D2E) d2e <- mkSizedFIFOF(2);
	FIFOF#(E2M) e2m <- mkSizedFIFOF(2);

	FIFO#(MemReq32) imemReqQ <- mkFIFO;
	FIFO#(Word) imemRespQ <- mkFIFO;
	FIFO#(MemReq32) dmemReqQ <- mkFIFO;
	FIFO#(Word) dmemRespQ <- mkFIFO;


	Reg#(Bit#(32)) cycles <- mkReg(0);
	Reg#(Bit#(32)) fetchCnt <- mkReg(0);
	Reg#(Bit#(32)) execCnt <- mkReg(0);

	Reg#(Bool) epoch_execute <- mkReg(False);
	Reg#(Bool) epoch_fetch <- mkReg(False);
	FIFOF#(Word) redirect_pcQ <- mkFIFOF;
	ScoreboardIfc#(8) sb <- mkScoreboard;
	Wire#(Bit#(32)) wireA <- mkDWire(32'hffffffff);
	Wire#(BypassTarget) forwardE <- mkDWire(BypassTarget{ dst: 0, data: 0 });

	rule incCycle;
		cycles <= cycles + 1;
	endrule

	rule doFetch; //(stage == Fetch);
		Word curpc = pc;
		
		Bool epoch = epoch_fetch;

		if ( redirect_pcQ.notEmpty ) begin
			redirect_pcQ.deq;
			curpc = redirect_pcQ.first;
			epoch = !epoch_fetch;
			epoch_fetch <= epoch;
		end

		Word predicted_pc = curpc + 4;
		//pc <= predicted_pc;
		pc <= curpc;
		//pc <= pc + 4;

		imemReqQ.enq(MemReq32{write:False,addr:truncate(pc),word:?,bytes:3});
		f2d.enq(F2D {pc: curpc, predicted_pc: predicted_pc, epoch: epoch});

		$write( "[0x%8x:0x%4x] Fetching instruction count 0x%4x\n", cycles, curpc, fetchCnt );
		fetchCnt <= fetchCnt + 1;
		//stage <= Decode;
	endrule





	rule doDecode; //(stage == Decode);
		let x = f2d.first;
		//f2d.deq;
		Word inst = imemRespQ.first;
		//imemRespQ.deq;

		Word rVal1;
		Word rVal2;

		let dInst = decode(inst);

		Bool stallSrc1 = sb.search1(dInst.src1);
		Bool stallSrc2 = sb.search2(dInst.src2);

		if ( forwardE.dst == dInst.src1 ) begin
			stallSrc1 = False;
			rVal1 = forwardE.data;
		end
		else begin
			rVal1 = rf.rd1(dInst.src1);
		end
		if ( forwardE.dst == dInst.src2 ) begin
			stallSrc2 = False;
			rVal2 = forwardE.data;
		end
		else begin
			rVal2 = rf.rd2(dInst.src2);
		end
		//let rVal1 = rf.rd1(dInst.src1);
		//let rVal2 = rf.rd2(dInst.src2);

		if ( !stallSrc1 && !stallSrc2 ) begin
			f2d.deq;
			imemRespQ.deq;
			sb.enq(dInst.dst);
			$write( "[0x%8x:0x%04x] Decoding 0x%08x\n", cycles, x.pc, inst );
		end
		else begin
			$write( "[0x%8x:0x%04x] Decode stalled -- %d %d\n", cycles, x.pc, dInst.src1, dInst.src2 );
		end

		d2e.enq(D2E {pc: x.pc, predicted_pc: x.predicted_pc, epoch: x.epoch, dInst: dInst, rVal1: rVal1, rVal2: rVal2});

		$write( "[0x%8x:0x%04x] decoding 0x%08x\n", cycles, x.pc, inst );
		//stage <= Execute;
	endrule






	rule doExecute; //(stage == Execute);
		D2E x = d2e.first; 
		d2e.deq;
		Word curpc = x.pc; 
		Word rVal1 = x.rVal1; Word rVal2 = x.rVal2; 
		DecodedInst dInst = x.dInst;

		let eInst = exec(dInst, rVal1, rVal2, curpc);

		if ( x.epoch == epoch_execute ) begin
			if ( eInst.nextPC != x.predicted_pc ) begin
				redirect_pcQ.enq(eInst.nextPC);
				epoch_execute <= !epoch_execute;
			end
        		if (eInst.iType == Unsupported) begin
        			$display("Reached unsupported instruction");
        			$display("Total Clock Cycles = %d\nTotal Instruction Count = %d", cycles, execCnt);
        			$display("Dumping the state of the processor");
        			$display("pc = 0x%x", x.pc);
        			//rf.displayRFileInSimulation;
        			$display("Quitting simulation.");
        			$finish;
        		end
        
        		if (eInst.iType == LOAD) begin
        			dmemReqQ.enq(MemReq32{write:False,addr:truncate(eInst.addr), word:?, bytes:dInst.size});
        			e2m.enq(E2M{dst:eInst.dst,extendSigned:dInst.extendSigned,size:dInst.size, pc:curpc, data:0, isMem: True});
        			//stage <= Writeback;
        			$write( "[0x%8x:0x%04x] \t\t Mem read from 0x%08x\n", cycles, curpc, eInst.addr );
        		end 
        		else if (eInst.iType == STORE) begin
        			dmemReqQ.enq(MemReq32{write:True,addr:truncate(eInst.addr), word:eInst.data, bytes:dInst.size});
        			$write( "[0x%8x:0x%04x] \t\t Mem write 0x%08x to 0x%08x\n", cycles, curpc, eInst.data, eInst.addr );
        			e2m.enq(E2M{dst:0,extendSigned:?,size:?, pc:curpc, data:?, isMem: False});
        			//stage <= Fetch;
        		end
        		else begin
        			if(eInst.writeDst) begin
        				e2m.enq(E2M{dst:eInst.dst,extendSigned:?,size:?, pc:curpc, data:eInst.data, isMem: False});
        				//stage <= Writeback;
        			end else begin
        				//stage <= Fetch;
        				e2m.enq(E2M{dst:0,extendSigned:?,size:?, pc:curpc, data:?, isMem: False});
        			end
        		end
		end
		else begin
			e2m.enq(E2M{ dst:0, extendSigned: ?, size: ?, pc:curpc, data: ?, isMem: False });
		end

		pc <= eInst.nextPC;
		forwardE <= BypassTarget{ dst: eInst.dst, data: eInst.data };

		execCnt <= execCnt + 1;
		$write( "[0x%8x:0x%04x] Executing\n", cycles, curpc );
		
	endrule







	rule doWriteback; //(stage == Writeback);
		e2m.deq;
		let r = e2m.first;

		sb.deq;

		Word dw = r.data;
		if ( r.isMem ) begin
			dmemRespQ.deq;
			let data = dmemRespQ.first;

			if ( r.size == 0 ) begin
				if ( r.extendSigned ) begin
					Int#(8) id = unpack(data[7:0]);
					Int#(32) ide = signExtend(id);
					dw = pack(ide);
				end else begin
					dw = zeroExtend(data[7:0]);
				end
			end else if ( r.size == 1 ) begin
				if ( r.extendSigned ) begin
					Int#(16) id = unpack(data[15:0]);
					Int#(32) ide = signExtend(id);
					dw = pack(ide);
				end else begin
					dw = zeroExtend(data[15:0]);
				end
			end else begin
				dw = data;
			end
		end
		
		$write( "[0x%8x:0x%04x] Writeback writing %x to %d\n", cycles, r.pc, dw, r.dst );
		rf.wr(r.dst, dw);

		
		//stage <= Fetch;
	endrule






	method ActionValue#(MemReq32) iMemReq;
		imemReqQ.deq;
		return imemReqQ.first;
	endmethod
	method Action iMemResp(Word data);
		imemRespQ.enq(data);
	endmethod
	method ActionValue#(MemReq32) dMemReq;
		dmemReqQ.deq;
		return dmemReqQ.first;
	endmethod
	method Action dMemResp(Word data);
		dmemRespQ.enq(data);
	endmethod
endmodule
