__description__ = 'Genetic algorithm to solve a scheduling problem of N jobs on M parallel and identical machines'
import random
import time



def random_chromosome(m, n):

    # Jobs assignment  : Boolean matrix with 1 line by job, 1 column by machine
    new_chromosome = [[0 for i in range(m)] for j in range(n)]
    # For each job, assign a random machine
    for i in range(n):
        new_chromosome[i][random.randint(0, m - 1)] = 1
    return new_chromosome


def fitness(chromosome, m, n, jobsProcessingTime):

    max_processing_time = -1
    for i in range(m):
        machine_processing_time = 0
        for j in range(n):
            machine_processing_time += chromosome[j][i] * jobsProcessingTime[j]
        # Save the maximum processing time found
        if machine_processing_time > max_processing_time:
            max_processing_time = machine_processing_time
    return max_processing_time


def crossover(chromosome1, chromosome2, m, n):

    new_chromosome = [[0 for i in range(m)] for j in range(n)]
    for i in range(n):
        # Alternate the pickup between the two selected solutions
        if not i % 2:
            new_chromosome[i] = chromosome1[i]
        else:
            new_chromosome[i] = chromosome2[i]
    return new_chromosome

def evolve(population, GA_POPSIZE, GA_ELITRATE, GA_MUTATIONRATE, m, n):

    new_population = [[] for i in range(GA_POPSIZE)]
    # First : Keep elites untouched
    elites_size = int(GA_POPSIZE * GA_ELITRATE)
    for i in range(elites_size):  # Elitism
        new_population[i] = population[i]
    # Then generate the new population
    for i in range(elites_size, GA_POPSIZE):
        # Generate new chromosome by crossing over two from the previous population
        new_population[i] = crossover(population[random.randint(0, GA_POPSIZE / 2)],
                                     population[random.randint(0, GA_POPSIZE / 2)], m, n)
        # Mutate
        if random.random() < GA_MUTATIONRATE:
            random_job = random.randint(0, n - 1)
            # Reset assignment
            new_population[i][random_job] = [0 for j in range(m)]
            # Random re-assignment
            new_population[i][random_job][random.randint(0, m - 1)] = 1
    return new_population


