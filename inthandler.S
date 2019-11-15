        .section        .text
        .global         fdi_nmi_int_handler

fdi_nmi_int_handler:

		sethi   %hi(OBC_Process), %l1
        or      %l1, %lo(OBC_Process), %l1
		sethi   %hi(OBC_Process + 0x4), %l2
        or      %l2, %lo(OBC_Process + 0x4), %l2

        sethi   %hi(FDI_NMI_Count), %g1
        or      %g1, %lo(FDI_NMI_Count), %g1
        ldub    [%g1],%g1
        add     %g1,1,%g1
        mov     %g1,%g2
        sethi   %hi(FDI_NMI_Count), %g1
        or      %g1, %lo(FDI_NMI_Count), %g1
        stb     %g2,[%g1]

		jmp     %l1
        rett    %l2
		
