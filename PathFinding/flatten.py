import time
def flatten(array):
	for x in range(len(array)):
		if type(array[x]) == list:
			return array[:x] + flatten(array[x]) + flatten(array[x+1:])
	return array

def flatten1(array):
	if isinstance(array, list):
		return sum(map(flatten1, array),[])

		return sum(queued, [])
	else:
		return [array]
