#!/usr/bin/env python3
import cv2 as cv
from pathlib import Path

class ImageHandler:
    def __init__(self, image_path,region_loader,data_reader):
        self.image = cv.imread(image_path)

        if self.image is None:
            raise ValueError(f"Error: Could not load image from {image_path}")

        self.region_loader = region_loader
        self.data_reader = data_reader
        self.year_img_list = []

    '''todo:
    +get rid of methods that are used one time or that are one line of code'''

    
    def draw_region(self,img, region_num, color):
        region = self.region_loader.get_region(region_num)
        cv.drawContours(img, region[0], -1, color, thickness=-1)

    def draw_images(self):
        years= self.data_reader.get_years()

        for year in years:
            try:
                row = self.data_reader.get_row(year)
                if row == "No Such year exists":
                    print(f"Skipping year {year}: No data found")
                    continue

                copy_img = self.image.copy()
                cv.putText(copy_img, year, (50,50), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 4)


                for i in range(len(row)):
                    if row[i]:
                        self.draw_region(copy_img, i, (0, 0, 255))
                    else:
                        pass

                base_dir = Path(__file__).resolve().parent.parent
                image_path = str(base_dir / "images")
                print(f"base dir: {base_dir}\nimage_path: {image_path}")
                cv.imwrite(image_path + year + ".png", copy_img)

                self.year_img_list.append(copy_img)

            except ValueError as e:
                print(f"Error processing year {year}: {str(e)}")
                continue  # Continue with the next year
            except Exception as e:
                print(f"Unexpected error processing year {year}: {str(e)}")
                continue  # Continue with the next year

        
        return self.year_img_list