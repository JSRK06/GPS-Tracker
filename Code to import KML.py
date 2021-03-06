from pykml import parser

lat = [] # variable to store latitude values
long = [] # variable to store longitude values

text_file_path = r'C:/Users/TAPL_PC_011/Documents/Arduino/Hand Held GPS/FINAL GPS TRACKER FILE/GPS.txt'
text_file_output_path = r'C:/Users/TAPL_PC_011/Documents/Arduino/Hand Held GPS/FINAL GPS TRACKER FILE/GPS_Output.txt'
kml_file_path= r'C:/Users/TAPL_PC_011/Documents/Arduino/Hand Held GPS/FINAL GPS TRACKER FILE/GPS_Data.kml'

def get_lat_long(kml_file):
    f = open(kml_file, "r")
    docs = parser.parse(f)
    # print(docs.getroot())
    try:
        doc = docs.getroot().Document.Folder
        # print(1)
    except:
        doc = docs.getroot().Document
        # print(2)

    # ===================================== Load KML and append the Data in variable ============================
    add = 0
    for place in doc.Placemark:
        x = str(place.Point.coordinates)
        x = x.split(',')
        long.append(x[0])
        lat.append(x[1])
        add += 1

    print('number of points are ' + str(add))
    return add


get_lat_long(kml_file_path)

# now lat and lon variables hold the values

# create string to replace in the code

long_string='{}'
lat_string='{}'
for i in range(len(long)):
    if i==len(long)-1:
        long_string=long_string[:-1]+long[i]+'}'
        lat_string=lat_string[:-1]+lat[i]+'}'
    else:
        long_string=long_string[:-1]+long[i]+', }'
        lat_string=lat_string[:-1]+lat[i]+', }'

# now long_string and lat_string will hold values to replace in the code

# code to change the text file

with open(text_file_path, 'rb') as f:
    text = f.readlines()
wr = open(text_file_output_path, 'w+b')
for t in text:
    # print(t)
    if t.find(b'float lat_away[]')==0:
        # print(t)
        val='float lat_away[] = '+lat_string+';  //changed value lat\r\n'
        val=bytes(val, 'utf8')
        # print(val)
        wr.write(val)
    elif t.find(b'float long_away[]')==0:
        # print(t)
        val='float long_away[] = '+long_string+';  //changed value lon\r\n'
        val=bytes(val, 'utf8')
        # print(val)
        wr.write(val)
    else:
        wr.write(t)

wr.close()

#the text file will be changed and willbe saved in the output_path