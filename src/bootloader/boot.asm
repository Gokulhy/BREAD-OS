org 0x7C00  ; Tell the assembler that the code will be loaded at memory address 0x7C00
bits 16     ; Generate 16-bit instructions

; FAT 12 header (BIOS Parameter Block - BPB)
; This section contains critical metadata about the filesystem structure on the disk.
; All multi-byte values (dw, dd) are stored in Little-Endian format (least significant byte first).

jmp short start
nop


bdb_oem:                    db 'MSWIN4.1'; OEM Identifier (8 bytes). Identifies the system that formatted the disk.
bdb_bytes_per_sector:       dw 512       ; Bytes per sector (2 bytes). Standard for floppy disks and most drives.
bdb_sectors_per_cluster:    db 1         ; Sectors per cluster (1 byte). Smallest unit of disk space allocation.
                                         ; For 1.44MB floppies, 1 sector/cluster is typical (512 bytes per cluster).
bdb_reserved_sectors:       dw 1         ; Number of reserved sectors (2 bytes). This includes the boot sector itself.
                                         ; Typically 1 for FAT12/16. These precede the FATs.
bdb_fat_count:              db 2         ; Number of File Allocation Tables (FATs) (1 byte). Two copies for redundancy.
bdb_dir_enteries_count:     dw 0E0H      ; Maximum number of root directory entries (2 bytes).
                                         ; 0xE0h (224 decimal) is standard for 1.44MB floppies.
                                         ; This implicitly determines the size of the root directory region.
bdb_total_sectors:          dw 2880      ; Total number of sectors in the logical volume (2 bytes).
                                         ; 2880 sectors = 1.44MB (2880 * 512 bytes/sector).
                                         ; Used if total sectors < 65536.
bdb_media_description_type: db 0F0h      ; Media descriptor type (1 byte). Defines the type of media.
                                         ; 0xF0h is standard for 1.44MB floppy disks.
bdb_sectors_per_fat:        dw 9         ; Number of sectors per FAT (2 bytes). For FAT12/FAT16 only.
                                         ; Each FAT occupies 9 sectors on a 1.44MB floppy.
bdb_sectors_per_track:      dw 18        ; Number of sectors per track (2 bytes). Physical geometry hint.
                                         ; 18 sectors/track for 1.44MB floppy.
bdb_heads:                  dw 2         ; Number of heads/sides on the storage media (2 bytes). Physical geometry hint.
                                         ; 2 heads for 1.44MB floppy.
bdb_hidden_sectors:         dd 0         ; Number of hidden sectors (4 bytes). LBA of the beginning of the partition.
                                         ; Typically 0 for floppies, as they are not partitioned.
bdb_large_sector_count:     dd 0         ; Large sector count (4 bytes). Used if bdb_total_sectors is 0 (i.e., > 65535 sectors).
                                         ; Since total sectors fit in 2 bytes, this is 0.


; Extended Boot Record (EBR) for FAT12/FAT16
; This section provides additional metadata following the standard BPB.

ebr_drive_number:           db 0         ; BIOS drive number (1 byte). 0x00 for floppy, 0x80 for hard disk.
                            db 0         ; Reserved byte (1 byte). Used by Windows NT for flags, otherwise reserved.
ebr_signature:              db 29h       ; Extended Boot Record signature (1 byte).Must be 0x28 or 0x29 to indicate validity of following fields.
ebr_volume_id:              db 12h, 34h, 56h, 78h ; Volume Serial Number (4 bytes). Unique ID for the volume.
ebr_volume_label:           db 'BREAD OS   ' ; Volume Label (11 bytes). Padded with spaces to 11 characters.
ebr_system_id:              db 'FAT12   ' ; File System Type String (8 bytes). Padded with spaces to 8 characters.


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

	mov [ebr_drive_number],dl 

	mov ax,1  			;LBA=1
	mov cl,1  			;1 sector to read
	mov bx,0x7E00		;data should be after bootloader
	call disk_read

	; Call the puts subroutine to print our message
	mov si, msg_hello 	; Load the address (offset) of msg_hello into SI
	call puts         	; Call the puts subroutine

	cli				  	;disable interrupts
	hlt 				; Halt the CPU - wait for interrupts.
        				; This is effectively the end of our boot sector's active execution.
floppy_error:
	mov si,read_msg_failed
	call puts
	jmp wait_key_and_reboot

wait_key_and_reboot:
	mov ah,0
	int 16h			; wait for keypress
	jmp 0FFFFh:0	;jump to the beginning of BIOS

.halt:
	cli 			; disable interrupts
	hlt

		
; lba_to_chs: Converts LBA to CHS format for BIOS disk access
; Input: ax = LBA, [bdb_sectors_per_track] = sectors per track, [bdb_heads] = number of heads
; Output: ch = cylinder (low 8 bits), cl = sector (bits 0-5) + cylinder high bits (6-7), dh = head, dl = drive number
lba_to_chs:
    push ax                ; Save ax register
    push dx                ; Save dx register

    xor dx, dx             ; Clear dx for division
    div word [bdb_sectors_per_track] ; ax = LBA / sectors per track, dx = remainder
    inc dx                 ; Sector = remainder + 1 (BIOS sectors are 1-based)
    mov cx, dx             ; Store sector in cx

    xor dx, dx             ; Clear dx for next division
    div word [bdb_heads]   ; ax = quotient (cylinder), dx = remainder (head)
    mov dh, dl             ; Store head number in dh. Since head numbers are small (< 256), they fit in dl (e.g., 0 or 1).
    mov ch, al             ; Store cylinder (low 8 bits) in ch. The div instruction places the cylinder number in ax. Since cylinders are small, al holds the relevant value.
    shl ah, 6              ; Shift cylinder high bits (8-9) to cl bits 6-7
    or cl, ah              ; Combine sector and cylinder high bits in cl

    pop ax                 ; Restore ax (contains drive number in al)
    mov dl, al             ; Set dl to drive number for BIOS
    pop ax                 ; Restore original ax
    ret                    ; Return with CHS in ch, cl, dh, dl


disk_read:
	push ax    ; Save LBA (from input)
    push bx    ; Save memory offset (from input)
    push cx    ; Save sectors to read (from input)
    push dx    ; Save drive number (from input)
    push di    ; Save DI (might be used internally or by BIOS)

	push cx
	call lba_to_chs
	pop ax

	 ; --- 3. Prepare for BIOS Disk Read (INT 13h, Function 02h) ---
    mov ah, 02h     ; Set AH register to 02h. This tells BIOS INT 13h that we want to 'Read Sectors'.
                    ; The necessary registers for INT 13h are:
                    ;   AH = 02h (read function)
                    ;   AL = number of sectors to read (from the 'pop ax' above)
                    ;   CH = cylinder (low 8 bits)   (from lba_to_chs)
                    ;   CL = sector + cylinder high bits (from lba_to_chs)
                    ;   DH = head                     (from lba_to_chs)
                    ;   DL = drive number             (from lba_to_chs)
                    ;   ES:BX = memory buffer address (from input parameter)
	mov di, 3		;retry counter.
.retry:
	pusha			;save all registers
	stc				;set carry flag
	int 13h			; Call BIOS Interrupt 13h. This is the instruction that tells the BIOS to
                    ; perform the disk read using the parameters in AH, AL, CX, DX, ES:BX.
	jnc .done 		;jump if carry flag not set
	
	popa			; Pop ALL general-purpose registers to restore their state from before the failed INT 13h call.
	call disk_reset

	dec di			; Decrement our retry counter.
	test di,di		; Test DI against itself. This sets the Zero Flag (ZF) if DI becomes 0.
	jnz .retry		; Jump if Not Zero (ZF = 0). If DI is not 0, it means we still have retries left.
.fail:
	jmp floppy_error ;jump to floppy_error if all 3 tries failed
.done:
	popa            ; Pop ALL general-purpose registers (matching the pusha in .retry)

    pop di          ; Restore DI (pushed at beginning of disk_read)
    pop dx          ; Restore DX (pushed at beginning of disk_read)
    pop cx          ; Restore CX (pushed at beginning of disk_read)
    pop bx          ; Restore BX (pushed at beginning of disk_read)
    pop ax          ; Restore AX (pushed at beginning of disk_read)
    ret             ; Return from subroutine
;reset disk_controller
; dl:drive number
disk_reset:
	pusha
	mov ah,0
	stc
	int 13h
	jc floppy_error
	popa
	ret
; --- Data Section ---
; Our null-terminated string to be printed.
; Using NEWLINE constant we defined earlier.
msg_hello: db 'BREAD OS', 0x0D, 0x0A, 0
read_msg_failed:db 'Read from disk failed',0x0D, 0x0A, 0
; --- Boot Sector Signature and Padding ---
; Fill the rest of the 512-byte sector with zeros.
; ($ - $$) calculates the current size of the code and data from the beginning (0x7C00).
; We need to fill up to byte 510 to make room for the 2-byte signature.
times 510-($-$$) db 0

; The boot sector signature (0xAA55) - must be at the very end of the 512-byte sector (offset 510-511).
dw 0AA55H
