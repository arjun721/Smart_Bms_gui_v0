speed = 10

while True:
    print("Current speed =", speed)
    cmd = input("Enter new speed (or q to quit): ")

    if cmd == "q":
        break

    if cmd.isdigit():
        speed = int(cmd)    # update variable at runtime
