# display images
from PIL import Image
import glob

def process():
    for imageName in glob.glob('images/*.jpeg'):
        basewidth = 640
        print("print")
        img = Image.open(imageName)
        wpercent = (basewidth/float(img.size[0]))
        hsize = int((float(img.size[1])*float(wpercent)))
        img = img.resize((basewidth,hsize), Image.NEAREST)
        img = img.convert("RGB")
        img.save(imageName)

if __name__== '__main__':
    process()