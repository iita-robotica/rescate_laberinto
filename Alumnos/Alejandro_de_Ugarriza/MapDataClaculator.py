def findAll(a_str, sub):
    start = 0
    while True:
        start = a_str.find(sub, start)
        if start == -1: return
        yield start
        start += len(sub) # use start += 1 to find overlapping matches


while True:
    print("")
    print("________________________________________________________")
    print("")
    directory = input("Enter the path to the .wbt file (q to quit): ")
    print("*****************")

    if directory == "q":
        break

    with open(directory,'r') as file:
        text = file.read()
    
    #Checkpoints
    nOfCheckpoints = text.count("checkpoint TRUE")
    nOfTraps = text.count("trap TRUE")
    nOfSwamps = text.count("swamp TRUE")
    scoreByCheckpoints = nOfCheckpoints * 10


    # Victims
    victimHeader = "Victim {"
    victimStartIndexes = list(findAll(text, victimHeader))
    victimEndIndexes = []
    for victimStartIndex in victimStartIndexes:
        victimEndIndex = text.find("}", victimStartIndex)
        victimEndIndexes.append(victimEndIndex)
    
    victims = []
    for vStartIndex, vEndIndex in zip(victimStartIndexes, victimEndIndexes):
        victim = text[vStartIndex + len(victimHeader): vEndIndex]
        scHeader = "scoreWorth "
        scoreIndex = victim.find(scHeader) 
        scoreIndex += len(scHeader)
        score = int(victim[scoreIndex:scoreIndex + 2])
        bonusPoints = 10

        if victim.count("type"):
            typeIndex = victim.find('type "') + len('type "')
            letter = victim[typeIndex].upper()
        else:
            letter = "T"

        victimDict = {
            "type":letter,
            "score":score,
            "bonus":bonusPoints
        }

        victims.append(victimDict)

    counts = {
        "H":0,
        "U":0,
        "S":0,
        "T":0
    }
    totalVictimScore = 0
    
    

    print("VICTIMS: ")
    for victim in victims:
        print(victim)
        counts[victim["type"]] += 1
        totalVictimScore += victim["score"] + victim["bonus"]
        print("------------------")

    finalScore = int((totalVictimScore + scoreByCheckpoints) * 1.1) + 10

    print("Quantity of each type of victim: ")
    for count in counts.items():
        print(count)
    
    print("Total score by victims: " + str(totalVictimScore))

    print("*****************")
    print("CHECKPOINTS")

    print("Number of checkpoints: " + str(nOfCheckpoints))
    print("Total score by checkpoints: " + str(scoreByCheckpoints))

    print("*****************")
    print("EXIT")

    print(f"extra 10% of final score: " + str(int((totalVictimScore + scoreByCheckpoints) * 0.1)))
    print("Bonus exit points: 10")
    
    print("*****************")
    print("FINAL SCORE")
    print("Maximum score : " + str(finalScore))

    print("*****************")
    print("DATA EXTRA")
    nOfObs = text.count('name "obstacle')
    print("Number of obstacles: " + str(nOfObs))
    print("Number of traps: " + str(nOfTraps))
    print("Number of swamps: " + str(nOfSwamps))
    
