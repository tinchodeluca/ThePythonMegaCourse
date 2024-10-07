def foo(side):
    return side**2

def foo(ounce):
    return (ounce * 29.57353 )

def foo(temp):
    if temp <= 7:
        return 'Cold'
    return 'Warm'

def foo(text):
    if 8 <= len(text):
        return (True)
    return(False)

def foo(temp):
    if 25 < temp: #if its higher than 25
        return 'Hot'
    elif 15 <= temp: #It will include up to 25
        return 'Warm'
    return 'Cold' #Other cases