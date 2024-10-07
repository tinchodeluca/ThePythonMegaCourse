def join_strings(first, second):
    #return f'{first}{second}'
    return first+second

def sort_list(*args):
    items = [item.upper() for item in args]
    items.sort()
    return items

def find_sum(**kwargs):
    print(kwargs)
    return sum(kwargs.values())
    
print(find_sum(x=1,w=2,e=3,r=3))