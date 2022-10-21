from asyncio.log import logger
import logging
import numpy as np
import pandas as pd
import statistics as s

#Logging
# logging.basicConfig(filename='Log\\filter_function.log',format='%(asctime)s || %(levelname)s || %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',level=logging.DEBUG)

def EuclideanDistance(vec1,vec2):
    E2Distance = np.sqrt( np.square(vec1[0]- vec2[0]) + np.square(vec1[1]- vec2[1]) + np.square(vec1[2]- vec2[2]))
    return E2Distance

def pivot_data_filter(Pivot_data,Filter,SD):
    sizepriorfiltering = Pivot_data.shape[0]
    print('Input sample size: ',sizepriorfiltering)
    print('Input sample mean, SD RMS error: ', s.mean(Pivot_data['IMD_abs_e_RMS']), ' ' , s.stdev(Pivot_data['IMD_abs_e_RMS']))

    if Filter == 'RMS':
        Pivot_data = Pivot_data.loc[(Pivot_data['IMD_abs_e_RMS'] >= (s.mean(Pivot_data['IMD_abs_e_RMS'])-SD*s.stdev(Pivot_data['IMD_abs_e_RMS']))) & (Pivot_data['IMD_abs_e_RMS'] <= (s.mean(Pivot_data['IMD_abs_e_RMS'])+SD*s.stdev(Pivot_data['IMD_abs_e_RMS'])))]
    elif Filter == 'IMD_Intersection':
        Pivot_data = Pivot_data.loc[(Pivot_data['IMD12_abs_e'] >= (s.mean(Pivot_data['IMD12_abs_e'])-SD*s.stdev(Pivot_data['IMD12_abs_e']))) & (Pivot_data['IMD12_abs_e'] <= (s.mean(Pivot_data['IMD12_abs_e'])+SD*s.stdev(Pivot_data['IMD12_abs_e'])))&(Pivot_data['IMD13_abs_e'] >= (s.mean(Pivot_data['IMD13_abs_e'])-SD*s.stdev(Pivot_data['IMD13_abs_e']))) & (Pivot_data['IMD13_abs_e'] <= (s.mean(Pivot_data['IMD13_abs_e'])+SD*s.stdev(Pivot_data['IMD13_abs_e'])))&(Pivot_data['IMD14_abs_e'] >= (s.mean(Pivot_data['IMD14_abs_e'])-SD*s.stdev(Pivot_data['IMD14_abs_e']))) & (Pivot_data['IMD14_abs_e'] <= (s.mean(Pivot_data['IMD14_abs_e'])+SD*s.stdev(Pivot_data['IMD14_abs_e'])))&(Pivot_data['IMD23_abs_e'] >= (s.mean(Pivot_data['IMD23_abs_e'])-SD*s.stdev(Pivot_data['IMD23_abs_e']))) & (Pivot_data['IMD23_abs_e'] <= (s.mean(Pivot_data['IMD23_abs_e'])+SD*s.stdev(Pivot_data['IMD23_abs_e'])))&(Pivot_data['IMD24_abs_e'] >= (s.mean(Pivot_data['IMD24_abs_e'])-SD*s.stdev(Pivot_data['IMD24_abs_e']))) & (Pivot_data['IMD24_abs_e'] <= (s.mean(Pivot_data['IMD24_abs_e'])+SD*s.stdev(Pivot_data['IMD24_abs_e'])))&(Pivot_data['IMD34_abs_e'] >= (s.mean(Pivot_data['IMD34_abs_e'])-SD*s.stdev(Pivot_data['IMD34_abs_e']))) & (Pivot_data['IMD34_abs_e'] <= (s.mean(Pivot_data['IMD34_abs_e'])+SD*s.stdev(Pivot_data['IMD34_abs_e'])))]
    else:
        print('Select Aropriate Filter')
    
    print('Filtered data size:',(Pivot_data.shape[0]/sizepriorfiltering)*100, 'percentage of input sample')
    print('Mean, SD RMS error: ', s.mean(Pivot_data['IMD_abs_e_RMS']), ' ' , s.stdev(Pivot_data['IMD_abs_e_RMS']))
    #logging.info('Filtered data size:',(Pivot_data.shape[0]/sizepriorfiltering)*100, 'percentage of input sample')
    logging.info(Pivot_data)
    
    return Pivot_data

    
def static_data(path):
    Static_data_location = path
    Static_data = pd.read_csv(Static_data_location)

    #use for NDI data
    # Static_data = Static_data.loc[(Static_data['State'] == 'OK') & (Static_data['State.1'] == 'OK') & (Static_data['State.2'] == 'OK') & (Static_data['State.3'] == 'OK') & (Static_data['State.4'] == 'OK')]

    #Declaring the column numbers containing individual fiducial marker data
    m1 = 15 #change to 14 if live data is used
    m2 = m1 + 3
    m3 = m2 + 3
    m4 = m3 + 3

    #Obtaining position values
    m1_position = Static_data.to_numpy()[:, m1:m1+3]
    m2_position = Static_data.to_numpy()[:, m2:m2+3]
    m3_position = Static_data.to_numpy()[:, m3:m3+3]
    m4_position = Static_data.to_numpy()[:, m4:m4+3]

    #appending Inter Marker Distance(IMD) to Static_data
    IMD12 = []
    IMD13 = []
    IMD14 = []
    IMD23 = []
    IMD24 = []
    IMD34 = []

    for i in range(Static_data.shape[0]):
        imd12 = EuclideanDistance(m1_position[i],m2_position[i])
        imd13 = EuclideanDistance(m1_position[i],m3_position[i])
        imd14 = EuclideanDistance(m1_position[i],m4_position[i])
        imd23 = EuclideanDistance(m2_position[i],m3_position[i])
        imd24 = EuclideanDistance(m2_position[i],m4_position[i])
        imd34 = EuclideanDistance(m3_position[i],m4_position[i])
        IMD12.append(imd12)
        IMD13.append(imd13)
        IMD14.append(imd14)
        IMD23.append(imd23)
        IMD24.append(imd24)
        IMD34.append(imd34)

    Static_data['IMD12'] = IMD12
    Static_data['IMD13'] = IMD13
    Static_data['IMD14'] = IMD14    
    Static_data['IMD23'] = IMD23    
    Static_data['IMD24'] = IMD24    
    Static_data['IMD34'] = IMD34
    logging.info(Static_data)
    return Static_data


def pivot_data(df,Static_data):
    Pivot_data = df

    #use for NDI data
    # Pivot_data = Pivot_data.loc[(Pivot_data['State'] == 'OK') & (Pivot_data['State.1'] == 'OK') & (Pivot_data['State.2'] == 'OK') & (Pivot_data['State.3'] == 'OK') & (Pivot_data['State.4'] == 'OK')]

    #Declaring the column numbers containing individual fiducial marker data
    m1 = 14
    m2 = m1 + 3
    m3 = m2 + 3
    m4 = m3 + 3

    # Obtaining position values
    m1_position = Pivot_data.to_numpy()[:, m1:m1+3]
    m2_position = Pivot_data.to_numpy()[:, m2:m2+3]
    m3_position = Pivot_data.to_numpy()[:, m3:m3+3]
    m4_position = Pivot_data.to_numpy()[:, m4:m4+3]

    #appending Inter Marker Distance(IMD), absolute errors(IMD_abs_e) and RMS errro value(IMD_abs_e_RMS) to Pivot_data
    IMD12 = []
    IMD13 = []
    IMD14 = []
    IMD23 = []
    IMD24 = []
    IMD34 = []

    for i in range(Pivot_data.shape[0]):
        imd12 = EuclideanDistance(m1_position[i],m2_position[i])
        imd13 = EuclideanDistance(m1_position[i],m3_position[i])
        imd14 = EuclideanDistance(m1_position[i],m4_position[i])
        imd23 = EuclideanDistance(m2_position[i],m3_position[i])
        imd24 = EuclideanDistance(m2_position[i],m4_position[i])
        imd34 = EuclideanDistance(m3_position[i],m4_position[i])
        IMD12.append(imd12)
        IMD13.append(imd13)
        IMD14.append(imd14)
        IMD23.append(imd23)
        IMD24.append(imd24)
        IMD34.append(imd34)

    Pivot_data['IMD12'] = IMD12
    Pivot_data['IMD13'] = IMD13
    Pivot_data['IMD14'] = IMD14    
    Pivot_data['IMD23'] = IMD23    
    Pivot_data['IMD24'] = IMD24    
    Pivot_data['IMD34'] = IMD34    

    Pivot_data['IMD12_abs_e'] = np.abs(Pivot_data['IMD12'] - s.mean(Static_data['IMD12']))
    Pivot_data['IMD13_abs_e'] = np.abs(Pivot_data['IMD13'] - s.mean(Static_data['IMD13']))
    Pivot_data['IMD14_abs_e'] = np.abs(Pivot_data['IMD14'] - s.mean(Static_data['IMD14']))
    Pivot_data['IMD23_abs_e'] = np.abs(Pivot_data['IMD23'] - s.mean(Static_data['IMD23']))
    Pivot_data['IMD24_abs_e'] = np.abs(Pivot_data['IMD24'] - s.mean(Static_data['IMD24']))
    Pivot_data['IMD34_abs_e'] = np.abs(Pivot_data['IMD34'] - s.mean(Static_data['IMD34']))
    Pivot_data['IMD_abs_e_RMS'] = np.sqrt((Pivot_data['IMD12_abs_e']**2 + Pivot_data['IMD13_abs_e']**2 + Pivot_data['IMD14_abs_e']**2 + Pivot_data['IMD23_abs_e']**2 + Pivot_data['IMD24_abs_e']**2 + Pivot_data['IMD34_abs_e']**2)/6)
    logging.info(Pivot_data)
    return Pivot_data