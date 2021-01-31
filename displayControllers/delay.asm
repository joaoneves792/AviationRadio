delay_1000ms:       ; subroutine for 1s delay
                    ; initialize counters
  ldi r18, 0xFF     ; 255
  ldi r24, 0xD3     ; 211
  ldi r25, 0x30     ; 48
inner_loop:
  subi  r18, 0x01   ; 1
  sbci  r24, 0x00   ; 0
  sbci  r25, 0x00   ; 0
  brne  inner_loop
  ret
