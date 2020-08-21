# File for scraping relavant information from Qualtrics survey. Input filename as a positional argument

import numpy as np
import pandas as pd
import time
import argparse

def main(args):

    csv_file = args.csv_file[0]

    start_time = time.time()

    df = pd.read_csv(csv_file)

    valid = []

    for numb in range(16,60):

        try :

            name = df['Q{}_1'.format(numb)]

            valid.append('Q{}_1'.format(numb))

        except Exception as e:
            # print("Found exception in {}.".format(e))
            pass

    output = []

    for column_name, column in df.transpose().iterrows(): 

        if column_name in valid:

            # for colelems in df.loc[1:,column_name]:
            #     print("normal are {}".format(colelems))
            #     if colelems == 'nan':
            #         print("nani! ARE {}".format(colelems))
            
            data = pd.DataFrame(df.loc[1:,column_name].dropna())

            out = data.to_csv(index=False)

            output.append(out)

        else: 
            pass


    # Converting to pandas and saving.

    df = pd.DataFrame(output)

    df.to_csv('scrapedfile.csv', index=False)

    # compression_opts = dict(method='zip',
                            # archive_name='output.csv')  

    # df.to_csv('scraped.zip', index=False,
    #           compression=compression_opts)
    

    print("Process took {} seconds. \n Finished converting data and storing into scrapedfile.csv".format(time.time() - start_time))


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Scrape info from csv file')
    parser.add_argument('csv_file', metavar='csv', type=str, nargs='+',
                        help='csv file from Qualtrics survey')

    args = parser.parse_args()

    main(args)