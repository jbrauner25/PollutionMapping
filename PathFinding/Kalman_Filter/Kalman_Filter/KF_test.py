from Kalman_Filter import *
from Data_Parser import *

data = dataParserUCR("130622-b")
data = [d for d in data if d[2] < 5]
data = convert_data_cart(data)
map = Map(90, 140, 500)

filtered_map = Kalman_Filter_UCR(map, data, None)
