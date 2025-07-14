org 0x7C00  ; Tell the assembler that the code will be loaded at memory address 0x7C00
bits 16     ; Generate 16-bit instructions

; --- Entry Point ---
start:
	jmp main ; Jump to the main routine

; --- Subroutines ---
; Puts (Print String) Subroutine
; Expects SI to point to a null-terminated string (e.g., 'Hello', 0)
puts:
	; Save registers that will be modified by this subroutine
	push si
	push ax

.loop:
	lodsb      ; Load byte from [DS:SI] into AL, then increment SI
	or al, al  ; Check if AL is zero (null terminator) - sets ZF if AL is 0
	jz .done   ; If AL is zero, string has ended, jump to .done

	; If not null, print the character using BIOS teletype output
	mov ah, 0x0E ; AH = 0x0E is BIOS Teletype Output function
	             ; AL already contains the character from lodsb
	int 0x10     ; Call BIOS Interrupt 10h (Video Services)

	jmp .loop  ; Continue looping to print the next character

.done:
	; Restore saved registers from the stack
	pop ax
	pop si
	ret        ; Return from the subroutine

; --- Main Program ---
main:
	mov ax, 0
	mov ds, ax
	mov es, ax
	
	; Set up the Stack Segment (SS) and Stack Pointer (SP).
    ; Stack grows downwards. Setting SP to 0x7C00 means the stack
	mov ss, ax
	mov sp, 0x7C00 

	; Call the puts subroutine to print our message
	mov si, msg_hello ; Load the address (offset) of msg_hello into SI
	call puts         ; Call the puts subroutine

	hlt ; Halt the CPU - wait for interrupts.
        ; This is effectively the end of our boot sector's active execution.



.halt:
	jmp .halt ; If an interrupt wakes up the CPU, this loop keeps it stuck here
              ; preventing it from executing random memory.

; --- Data Section ---
; Our null-terminated string to be printed.
; Using NEWLINE constant we defined earlier.
msg_hello: db 'BREAD OS', 0x0D, 0x0A, 0

; --- Boot Sector Signature and Padding ---
; Fill the rest of the 512-byte sector with zeros.
; ($ - $$) calculates the current size of the code and data from the beginning (0x7C00).
; We need to fill up to byte 510 to make room for the 2-byte signature.
times 510-($-$$) db 0

; The boot sector signature (0xAA55) - must be at the very end of the 512-byte sector (offset 510-511).
dw 0AA55H
