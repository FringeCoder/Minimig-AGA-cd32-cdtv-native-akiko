; Test program for the TG68K savestate-restore testbench.
;
; Two independent pieces of code:
;
;   main     -- what the machine is "running" when the savestate is taken.
;               Spins forever, touching D0/D1 and a memory marker so the
;               testbench can see the CPU is alive before it freezes it.
;
;   restored -- where a restore is supposed to land. Writes the restored D3,
;               A2 and a fixed tag to memory. Every one of those writes is a
;               claim the testbench checks: reaching them at all proves the
;               CPU resumed at the restored PC, and their values prove the
;               register file survived the restore.
;
; The main loop is deliberately not three instructions. The testbench freezes
; the CPU at every clock across one pass of it, so whatever the loop contains
; decides which mid-instruction states a restore ever has to survive. It needs:
;
;   movem     multi-word transfers, so movem_run / memmask are mid-sequence
;   lsl #7    a rotate count, so rot_cnt is mid-sequence
;   move (a0) an operand staged through the address registers
;   bsr / rts a return address on the stack and a PC loaded from memory,
;             which is the only thing that puts exec(directPC) in flight
;   jmp       a PC loaded from an effective address, i.e. exec(ea_to_pc)
;
; The last two matter most: without them nothing in the loop ever leaves a
; decoded "write the PC from somewhere" in exec, and a resume that forgets to
; clear exec would look correct.
;
; Assemble with:
;   vasmm68k_mot -Fbin -o tg68k_ss_prog.bin tg68k_ss_prog.s
; then convert to the word hex the testbench reads (see build.sh).

MAIN_MARK   EQU     $600            ; main loop's heartbeat
RES_D3      EQU     $604            ; D3 as seen after the restore
RES_A2      EQU     $608            ; A2 as seen after the restore
RES_TAG     EQU     $60C            ; fixed tag, proves the third instruction ran
SCRATCH     EQU     $610            ; main loop's read-modify-write target

            ORG     $0
            DC.L    $00007FF0       ; reset SSP
            DC.L    main            ; reset PC

;-----------------------------------------------------------------------------
; The running program.
;-----------------------------------------------------------------------------
            ORG     $400
main:
            move.l  #$11111111,d0
            move.l  #$22222222,d1
            lea     SCRATCH,a0
main_loop:
            addq.l  #1,d0
            move.l  d0,MAIN_MARK
            movem.l d0-d1/a0,-(a7)
            movem.l (a7)+,d0-d1/a0
            lsl.l   #7,d1
            move.l  (a0),d2
            addq.l  #1,d2
            move.l  d2,(a0)
            bsr.s   sub1
            jmp     cont
cont:
            bra.s   main_loop
sub1:
            addq.l  #1,d1
            rts

;-----------------------------------------------------------------------------
; The restore target. First instruction is move.l d3,RES_D3 -- opcode $21C3,
; which the testbench matches against the word actually fetched.
;-----------------------------------------------------------------------------
            ORG     $500
restored:
            move.l  d3,RES_D3
            move.l  a2,RES_A2
            move.l  #$DEADBEEF,RES_TAG
rhalt:
            bra.s   rhalt
