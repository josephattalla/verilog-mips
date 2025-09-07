start:  xori $s0, $zero, 0x0003   # s0 = 3
        xori $s1, $zero, 0x0004   # s1 = 4
        j next1
next2:  xori $s0, $zero, 0x0001   # s0 = 1
        xori $s1, $zero, 0x0001   # s1 = 1
next1:  sub $s2, $s1, $s0         # s2 = s1 - s0
        bne $s0, $s1, next2       # if s0 != s1 goto next2
        add $s3, $s0, $s1         # s3 = s0 + s1
        sw $s3, 16($s2)
        lw $s4, 16($s2)
        slt $s5, $s0, $s4         # s5 = s0 < s4
        lw $s3, 16($s2)
        xori $s3, $s2, 0x0001     # s3 = s2 ^ 1
        xori $s5, $s5, 0x0001     # s5 ^= 1
        jr $s5
