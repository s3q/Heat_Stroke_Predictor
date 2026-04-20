def threeSmallest(lst):

    smallest = float('inf')
    second = float('inf')
    third = float('inf')

    for num in lst:

        if num < smallest:
            third = second
            second = smallest
            smallest = num

        elif num < second:
            third = second
            second = num

        elif num < third:
            third = num

    return [smallest, second, third]


print(threeSmallest([9,4,7,2,8,1]))
