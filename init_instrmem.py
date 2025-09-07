with open("instr.txt", "w") as f:
    for i in range(1024):
        for j in range(32):
            f.write("0")
        f.write("\n")

