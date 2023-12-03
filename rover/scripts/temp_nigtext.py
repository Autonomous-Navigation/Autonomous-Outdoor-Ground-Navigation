import pyfiglet

text = "SS  L   R"
ascii_art = pyfiglet.figlet_format(text, font="block")
print(ascii_art)


text =  "<-          |  <- ->  |  <- ^ ->"
ascii_art = pyfiglet.figlet_format(text,  font="small", width=100)
print(ascii_art)

from art import text2art

def direction_graphics_callback():
    text = "SS L R"

    # Print the original text
    print(text)

    # Generate ASCII art using the block font
    ascii_art = text2art(text, font='block')

    # Print the ASCII art
    print(ascii_art)

# Assuming the rest of your code remains unchanged
direction_graphics_callback()