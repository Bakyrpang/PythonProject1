morse = {'a':". -", 'b':"- . . .", 'c':"- . - .", 'd':"- . .", ' ':"/"}

msg = input("Enter the message: ")
new_msg = ""
for letter in msg:
    new_msg += morse[letter]
    new_msg += "   "

print(new_msg)