def foo(items):
    numbers = []
    strings = []
    
    for item in items:
        if isinstance(item, int):
            numbers.append(item)
        else:
            strings.append(item)
    return numbers

##return [i for i in lst if  isinstance(i, int)]

def foo(items):
    return [item for item in items if 0 < item]

def foo(items):
    return [item if not isinstance(item, str) else 0 for item in items]

def foo(items):
    return sum([float(item) for item in items])