from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, width=72, height=40)

font = ImageFont.load_default()

img = Image.new("1", (device.width, device.height))
draw = ImageDraw.Draw(img)

draw.text((0, 0),  "Hello", font=font, fill=255)
draw.text((0, 16), "World!", font=font, fill=255)

device.display(img)
