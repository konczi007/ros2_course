import turtle

t = turtle.Turtle()

def koch(length, step):
    if (step == 0):
        t.forward(length)
    else:
        length = length / 3
        step = step - 1
        koch(length, step)
        t.right(60)
        koch(length, step)
        t.left(120)
        koch(length, step)
        t.right(60)
        koch(length, step)
        
        
t.speed(20)
t.penup()
t.setposition(-150, -150)
t.pendown()
for index in range(3):
    koch(200, 3)
    t.left(120)

