import os
import glob
import pandas as pd
import xml.etree.ElementTree as ET

SKIP_NEGATIVES = True
NEGATIVE_CLASS = "none"

def xml_to_csv(path, skipNegatives):
    xml_list = []
    for xml_file in glob.glob(path + '/*.xml'):
        tree = ET.parse(xml_file)
        root = tree.getroot()
        if root.find('object'):
            for member in root.findall('object'):
                bbx = member.find('bndbox')                
                xmin = round(float(bbx.find('xmin').text))
                ymin = round(float(bbx.find('ymin').text))
                xmax = round(float(bbx.find('xmax').text))
                ymax = round(float(bbx.find('ymax').text))
                label = member.find('name').text
                value = (root.find('filename').text,
                        int(root.find('size')[0].text),
                        int(root.find('size')[1].text),
                        label,
                        xmin,
                        ymin,
                        xmax,
                        ymax
                        )
                print(value)
                xml_list.append(value)
        elif not skipNegatives:
            value = (root.find('filename').text,
                        int(root.find('size')[0].text),
                        int(root.find('size')[1].text),
                        NEGATIVE_CLASS,
                        0,
                        0,
                        0,
                        0
                        )
            print(value)
            xml_list.append(value)

    column_name = ['filename', 'width', 'height',
                   'class', 'xmin', 'ymin', 'xmax', 'ymax']
    xml_df = pd.DataFrame(xml_list, columns=column_name)
    return xml_df


def main():
    datasets = ['train', 'valid', 'test']

    for ds in datasets:
        image_path = os.path.join(os.getcwd(), 'images', ds)
        xml_df = xml_to_csv(image_path, SKIP_NEGATIVES)
        xml_df.to_csv('data/{}_data.csv'.format(ds), index=None)
        print('Successfully converted xml to csv.')


main()
