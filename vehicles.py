class missile:
    def __init__(self, missilename):
        self.missilename = missilename
        self.fuelmass = np.zeros(5)
        self.m0 = np.zeros(5)
        self.Isp0 = np.zeros(5)
        self.burntime = np.zeros(5)
        self.dMdt = np.zeros(5)
        self.etad = np.zeros(5)
        self.etar = np.zeros(5)
        self.tmin = np.zeros(5)
        self.tmax = np.zeros(5)

        if self.missilename == "SCUD":
            self.missiletype = 3
            self.payload = 987
            self.NumStages = 1
            #etad[1] = 29; tmin[1] = 10; tmax[1] = 20
            self.etad[1] = 16; self.tmin[1] = 10; self.tmax[1] = 20  #Scud MET

            #STAGE 1:
            self.fuelmass[1] = 3771
            self.m0[1] = 4873                    
            self.Isp0[1] = 226 * 9.8                     #in m/s; sea level value
            self.burntime[1] = 62                        #in secs
            self.dMdt[1] = 58.8 

            self.maxdiam = .88                           #in m; max diam of missile
            self.nozarea = .1352                         #m2; exit area of nozzle
            self.beta = 4000                             #in lb/ft^2
            self.mshroud = 0
            self.tshroud = 0
            self.correction_time=0                #Time for stellar correction
            if stellar==True:
                print("Invalid Parameters for Stellar Correction")

        elif self.missilename == "SCUD-ER":
            self.missiletype = 3
            self.payload = 500
            self.NumStages = 1
            self.etad[1] = 2.7; self.tmin[1] = 10; self.tmax[1] = 20
            #etad[1] = 1.15; tmin[1] = 10; tmax[1] = 20  # MET

            #STAGE 1:
            self.fuelmass[1] = 7730     #  4.3% unburned
            self.m0[1] = 8730                    
            self.Isp0[1] = 230 * 9.8                     #in m/s; sea level value
            self.burntime[1] = 127.8                        #in secs
            self.dMdt[1] = 57.83 

            self.maxdiam = 1                           #in m; max diam of missile
            self.nozarea = .135                         #m2; exit area of nozzle
            self.beta = 1500                             #in lb/ft^2
            self.mshroud = 0
            self.tshroud = 0
            self.correction_time=0                #Time for stellar correction
            if stellar==True:
                print("Invalid Parameters for Stellar Correction")

        elif self.missilename == "Minuteman III":
            self.missiletype = 1
            self.payload = 900   # one W78 + bus
            self.NumStages = 3
            self.etad[1] = 18; self.tmin[1] = 10; self.tmax[1] = 25  #eta1= 18 is Met
            self.etad[2] = 30; self.tmin[2] = 66; self.tmax[2] = 96
            self.etad[3] = 45.7; self.tmin[3]= 118; self.tmax[3] = 148

            #STAGE 1:
            self.fuelmass[1] = 20780
            self.m0[1] = 23230
            self.Isp0[1] = 267 * 9.8                       #in m/s; sea level value
            self.burntime[1] = 61                       #in secs
            self.dMdt[1] = self.fuelmass[1] / self.burntime[1]    

            #Stage 2:
            self.fuelmass[2] = 6240
            self.m0[2] = 7270
            self.Isp0[2] = 287 * 9.8
            self.burntime[2] = 66
            self.dMdt[2] = self.fuelmass[2] / self.burntime[2]

            #Stage 3:
            self.fuelmass[3] = 3306
            self.m0[3] = 3710
            self.Isp0[3] = 285 * 9.8
            self.burntime[3] = 61
            self.dMdt[3] = self.fuelmass[3] / self.burntime[3]

            self.maxdiam = 1.7                       # max diam of missile (in m)
            self.nozarea = .68                       # nozzle area (in m2)  Derived from Isp(sl)=267 and Isp(vac)=287
            self.beta = 2000     #2500                    # ballistic coeff (in lb/ft2); To convert: x lb/ft2 = 47.9x N/m2
            self.mshroud = 150                       # shroud mass (in kg) -from Spaceflight101
            self.tshroud = 76                        # time of shroud release (in s)
            self.correction_time=0                #Time for stellar correction
            if stellar==True:
                print("Invalid Parameters for Stellar Correction") 
        
        elif self.missilename == "GBSD":
            self.missiletype = 1
            self.payload = 600   #3 W78 + bus  Standard payload = 1,000 kg --> 9000 km
            self.NumStages = 3
            self.etad[1] = 18.5; self.tmin[1] = 10; self.tmax[1] = 25
            self.etad[2] = 0; self.tmin[2] = 66; self.tmax[2] = 96
            self.etad[3] = 0; self.tmin[3]= 118; self.tmax[3] = 148

            #STAGE 1:
            self.m0[1] = 23230
            self.fuelmass[1] = .89*self.m0[1] #20780
            self.Isp0[1] = 267 * 9.8                       #in m/s; sea level value
            self.burntime[1] = 61                       #in secs
            self.dMdt[1] = self.fuelmass[1] / self.burntime[1]    

            #Stage 2:

            self.m0[2] = 7270
            self.fuelmass[2] = .86*self.m0[2] #6240    
            self.Isp0[2] = 287 * 9.8
            self.burntime[2] = 66
            self.dMdt[2] = self.fuelmass[2] / self.burntime[2]

            #Stage 3:
            self.m0[3] = 3710
            self.fuelmass[3] = .89*self.m0[3] #3306
            self.Isp0[3] = 285 * 9.8
            self.burntime[3] = 61
            self.dMdt[3] = self.fuelmass[3] / self.burntime[3]

            self.maxdiam = 1.7                       # max diam of missile (in m)
            self.nozarea = .68                       # nozzle area (in m2)  Derived from Isp(sl)=267 and Isp(vac)=287
            self.beta = 2500                         # ballistic coeff (in lb/ft2); To convert: x lb/ft2 = 47.9x N/m2
            self.mshroud = 150                       # shroud mass (in kg) -from Spaceflight101
            self.tshroud = 76                        # time of shroud release (in s)
            self.correction_time=3000                #Time for stellar correction 

        elif self.missilename == "SM-3":
            self.missiletype = 1
            self.payload = 50
            self.NumStages = 3
            self.etad[1] = 18.5; self.tmin[1] = 10; self.tmax[1] = 25
            self.etad[2] = 0; self.tmin[2] = 66; self.tmax[2] = 96
            self.etad[3] = 0; self.tmin[3]= 118; self.tmax[3] = 148

            #STAGE 1:
            self.m0[1] = 600
            self.fuelmass[1] = 600*.75
            self.Isp0[1] = 230 * 9.8                       #in m/s; sea level value
            self.burntime[1] = 6                       #in secs
            self.dMdt[1] = self.fuelmass[1] / self.burntime[1]    

            #Stage 2:

            self.m0[2] = 900
            self.fuelmass[2] = .84*self.m0[2]   
            self.Isp0[2] = 240 * 9.8
            self.burntime[2] = 60
            self.dMdt[2] = self.fuelmass[2] / self.burntime[2]

            #Stage 3:
            self.m0[3] = 270
            self.fuelmass[3] = .83*self.m0[3]
            self.Isp0[3] = 245 * 9.8
            self.burntime[3] = 60
            self.dMdt[3] = self.fuelmass[3] / self.burntime[3]

            self.maxdiam = .53                       # max diam of missile (in m)
            self.nozarea = .2                       # nozzle area (in m2)  Derived from Isp(sl)=267 and Isp(vac)=287
            self.beta = 500                         # ballistic coeff (in lb/ft2); To convert: x lb/ft2 = 47.9x N/m2
            self.mshroud = 20                       # shroud mass (in kg) -from Spaceflight101
            self.tshroud = 100                        # time of shroud release (in s)
            self.correction_time=950                 #Time for stellar correction

        elif self.missilename == "D5":
            self.missiletype = 1
            self.payload = 1000
            self.NumStages = 3
            self.etad[1] = 18.5; self.tmin[1] = 10; self.tmax[1] = 25
            self.etad[2] = 0; self.tmin[2] = 66; self.tmax[2] = 96
            self.etad[3] = 0; self.tmin[3]= 118; self.tmax[3] = 148

            #STAGE 1:
            self.m0[1] = 20000 #39241
            self.fuelmass[1] = 20000 #33355
            self.Isp0[1] = 281 * 9.8                       #in m/s; sea level value
            self.burntime[1] = 63                       #in secs
            self.dMdt[1] = self.fuelmass[1] / self.burntime[1]    

            #Stage 2:

            self.m0[2] = 11866
            self.fuelmass[2] = 10320   
            self.Isp0[2] = 281 * 9.8
            self.burntime[2] = 64
            self.dMdt[2] = self.fuelmass[2] / self.burntime[2]

            #Stage 3:
            self.m0[3] = 2191
            self.fuelmass[3] = 1970
            self.Isp0[3] = 281 * 9.8
            self.burntime[3] = 43
            self.dMdt[3] = self.fuelmass[3] / self.burntime[3]

            self.maxdiam = 2.11                       # max diam of missile (in m)
            self.nozarea = .68                       # nozzle area (in m2)  Derived from Isp(sl)=267 and Isp(vac)=287
            self.beta = 2000                         # ballistic coeff (in lb/ft2); To convert: x lb/ft2 = 47.9x N/m2
            self.mshroud = 150                       # shroud mass (in kg) -from Spaceflight101
            self.tshroud = 75                        # time of shroud release (in s)
            self.correction_time=200                 #Time for stellar correction

        else:
            print("Missile not found")

        self.area = (self.maxdiam / 2) ** 2 * np.pi             # cross-sectional area of missile (in m2)
        self.mtot = 0
        self.burntimetot = 0
        for i in range(1, self.NumStages+1):
            self.mtot = self.mtot + self.m0[i]                        # calculate total body mass
            self.burntimetot = self.burntimetot + self.burntime[i]    # calculate total burntime
        for i in range(1, 4):
            self.etar[i] = -self.etad[i] * np.pi / 180         # convert to radians and add correct sign
        self.mtot = self.mtot + self.payload + self.mshroud
        # convert to mks units:
        self.betaog = self.beta * (3.28) ** 2 / 2.2046            # in kg/m**2

        # INPUT DATA FOR DRAG FUNCTIONS DURING BOOST:
        # Drag coefficient for large solid missile, like MM, Trident*******
        self.drag_solid_tab = np.array([
            [0, .1007],
            [.5, .1099],
            [.7, .1286],
            [.8, .1486],
            [.9, .1825],
            [1, .2402],
            [1.05, .2814],
            [1.1, .3229],
            [1.15,.3535],
            [1.2, .3618],
            [1.4, .3358],
            [1.6, .3117],
            [1.8, .2891],
            [2, .2682],
            [2.5, .2226],
            [3, .1859],
            [3.5, .1571],
            [4, .1354],
            [4.5, .1198],
            [5.5, .1035],
            [6, .1010],
            ])

        if self.missiletype == 1:
            self.mach_tab = self.drag_solid_tab[:,0]                # array of mach numbers
            self.Cd_tab =self.drag_solid_tab[:,1]               # array of drag coeffs

        # Drag coefficient for large liquid missile, like Atlas, Titan*******
        self.drag_liq_tab = np.array([
            [0, .2007],
            [.7, .2286],
            [.8, .2486],
            [.9, .2825],
            [1, .3402],
            [1.05, .3838],
            [1.1, .4304],
            [1.15,.4654],
            [1.18, .4747],
            [1.2, .4740],
            [1.4, .4374],
            [1.6, .4034],
            [1.8, .3720],
            [2, .3430],
            [2.5, .2804],
            [3, .2305],
            [3.5, .1918],
            [4, .1626],
            [4.5, .1416],
            [5.5, .1183],
            [6, .1134],
            ])

        if self.missiletype == 2:
            self.mach_tab = self.drag_liq_tab[:,0]                 # array of mach numbers
            self.Cd_tab = self.drag_liq_tab[:,1]                # array of drag coeffs

        # Drag coefficient for V2 missile*******
        #    from Sutton, "Rocket Propulsion Elements" (3rd ed), pg. 119
        self.drag_V2_tab = np.array([
            [0, .15],
            [.6, .15],
            [.7, .16],
            [.8, .18],
            [.9, .23],
            [1, .3],
            [1.1, .4],
            [1.18,.42],
            [1.2, .415],
            [1.3, .38],
            [1.4, .36],
            [1.5, .325],
            [1.6, .3],
            [1.7, .275],
            [1.8, .26],
            [1.9, .25],
            [2, .24],
            [3, .21],
            [4, .18],
            [5, .155],
            [5.5, .15],
            ])

        if self.missiletype == 3:
            self.mach_tab = self.drag_V2_tab[:,0]                 # array of mach numbers
            self.Cd_tab = self.drag_V2_tab[:,1]                # array of drag coeffs
