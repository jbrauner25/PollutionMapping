import json
import urllib

ID_list = [
    '3479', '4670', '4692', '4725', '3515', '4674', '1684', '5184', '5080', '4689', '3617',
    '4686', '4734', '3551', '4721', '3487', '5228', '4662', '3529', '3505', '3525', '2448'
]

url_base = 'https://www.purpleair.com/json?show='
url_list = [url_base + i for i in ID_list]

sensor_dicts = []

for u in url_list:
    with urllib.request.urlopen(u) as url:
        dat = json.loads(url.read().decode())
        sensor_dicts.append(dat.get('results')[0])

for i in sensor_dicts:
    print(i)

