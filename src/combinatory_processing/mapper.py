def switch(if_true, if_false):
    return lambda k: if_true if k else if_false

def threshold(thresh_value):
    return lambda k: k >= thresh_value

def deadzone(deadzone):
    return lambda k: k if abs(k) >= deadzone else 0
