def o_Coverage(distancesarray):
    sum_one_over_distance = 0
    for x in range(len(distancesarray[0]) - 1):
        min_dist = 999999999999
        for y in range(len(distancesarray)):
            current_distance = distancesarray[y][x]
            if distancesarray[y][x] < min_dist:
                min_dist = current_distance
        sum_one_over_distance += 1 / min_dist  # Maximize this
    objective = sum_one_over_distance / len(distancesarray[0])
    return objective

def o_IntelligentSampling_incremental(cellset, node_objective, cell_loc, prev_sum, prev_len):
    if cell_loc in cellset:
        return prev_sum / prev_len, prev_sum, prev_len
    else:
        return (node_objective + prev_sum) / (prev_len+1), node_objective + prev_sum, prev_len + 1

def o_IntelligentSampling(cellset, node_objective, cell_loc, prev_sum, cell_size):
    if cell_loc in cellset:
        return prev_sum / cell_size, prev_sum
    else:
        return (node_objective + prev_sum) / cell_size, node_objective + prev_sum
