class DetectedObjectType(Enum):
    Any = 255
    Bridge = 15
    Buildings = 1
    Dynamic = 20
    Fences = 2
    Ground = 14
    GuardRail = 17
    names = {
        'Any': 255,
        'Bridge': 15,
        'Buildings': 1,
        'Dynamic': 20,
        'Fences': 2,
        'Ground': 14,
        'GuardRail': 17,
        'NONE': 0,
        'Other': 3,
        'Pedestrians': 4,
        'Poles': 5,
        'RailTrack': 16,
        'RoadLines': 6,
        'Roads': 7,
        'Sidewalks': 8,
        'Sky': 13,
        'Static': 19,
        'Terrain': 22,
        'TrafficLight': 18,
        'TrafficSigns': 12,
        'Vegetation': 9,
        'Vehicles': 10,
        'Walls': 11,
        'Water': 21,
    }
    NONE = 0
    Other = 3
    Pedestrians = 4
    Poles = 5
    RailTrack = 16
    RoadLines = 6
    Roads = 7
    Sidewalks = 8
    Sky = 13
    Static = 19
    Terrain = 22
    TrafficLight = 18
    TrafficSigns = 12
    Vegetation = 9
    Vehicles = 10
    Walls = 11
    Water = 21