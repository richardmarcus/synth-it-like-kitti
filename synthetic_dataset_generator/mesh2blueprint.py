## map mesh name to carla blueprint

def get_blueprint_from_mesh(meshname: str):
    '''
    Returns matching blueprint name for meshname input
    '''

    if "AudiA2" in meshname:
        return "vehicle.audi.a2"
    elif "Etron" in meshname:
        return "vehicle.audi.etron"
    elif "AudiTT" in meshname:
        return "vehicle.audi.tt"
    elif "BMWGrandTourer" in meshname:
        return "vehicle.bmw.grandtourer"
    elif "ChevroletImpala" in meshname:
        return "vehicle.chevrolet.impala"
    elif "Citroen_C3" in meshname:
        return "vehicle.citroen.c3"
    elif "DodgeCharger" in meshname:
        return "vehicle.dodge.charger_police"
    elif "ChargerCop" in meshname:
        return "vehicle.dodge.charger_police_2020"
    elif "Charger" in meshname:
        return "vehicle.dodge.charger_2020"
    elif "FordCrown" in meshname:
        return "vehicle.ford.crown"
    elif "Mustang" in meshname:
        return "vehicle.ford.mustang"
    elif "JeepWranglerRubicon" in meshname:
        return "vehicle.jeep.wrangler_rubicon"
    elif "LincolnMkz2017" in meshname:
        return "vehicle.lincoln.mkz_2017"
    elif "Lincoln" in meshname:
        return "vehicle.lincoln.mkz_2020"
    elif "MercedesBenzCoupeC" in meshname:
        return "vehicle.mercedes.coupe"
    elif "MercedesCCC" in meshname:
        return "vehicle.mercedes.coupe_2020"
    elif "BMWIsetta" in meshname:
        return "vehicle.micro.microlino"
    elif "MiniCooperS" in meshname:
        return "vehicle.mini.cooper_s"
    elif "Mini2021" in meshname:
        return "vehicle.mini.cooper_s_2021"
    elif "NissanMicra" in meshname:
        return "vehicle.nissan.micra"
    elif "NissanPatrolST" in meshname:
        return "vehicle.nissan.patrol"
    elif "NissanPatrol2021" in meshname:
        return "vehicle.nissan.patrol_2021"
    elif "SeatLeon" in meshname:
        return "vehicle.seat.leon"
    elif "TeslaM3" in meshname:
        return "vehicle.tesla.model3"
    elif "ToyotaPrius" in meshname:
        return "vehicle.toyota.prius"
    else:
        return "other"
