      LD0 00h
      SWP
      LD0 01h
      SWP
loop: ADD
      SWP
      ; Here we have result in R0
      JMP loop

