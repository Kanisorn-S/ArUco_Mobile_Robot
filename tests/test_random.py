result = 0
def read_sensor():
    print("type 1 to exit the loop: ")
    result = input()

while result == 0:
    read_sensor()
    print("result is currently : " + str(result))
print("exit from loop")

