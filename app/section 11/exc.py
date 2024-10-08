with open('bear.txt') as file:
    content = file.read()

print(content[:90])

def file_process(char, path):
    with open(path) as file:
        strings = file.read()
    return strings.count(char)

with open('file.txt',"w") as file:
    file.write('snail')