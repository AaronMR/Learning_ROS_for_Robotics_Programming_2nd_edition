#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>

geometry_msgs::Point global_position;

ros::Publisher position_pub;

const double RADIANS_PER_DEGREE = M_PI/180.0;
const double DEGREES_PER_RADIAN = 180.0/M_PI;

// WGS84 Parameters
const double WGS84_A = 6378137.0;                // major axis
const double WGS84_B = 6356752.31424518;        // minor axis
const double WGS84_F = 0.0033528107;                // ellipsoid flattening
const double WGS84_EP = 0.0820944379;                // second eccentricity

const double WGS84_E = 0.0818191908;                // first eccentricity
// UTM Parameters
const double UTM_K0 = 0.9996;                        // scale factor
const double UTM_FE = 500000.0;                // false easting
const double UTM_FN_N = 0.0;                        // false northing on north hemisphere
const double UTM_FN_S = 10000000.0;                // false northing on south hemisphere
const double UTM_E2 = (WGS84_E*WGS84_E);        // e^2
const double UTM_E4 = (UTM_E2*UTM_E2);                // e^4
const double UTM_E6 = (UTM_E4*UTM_E2);                // e^6
const double UTM_EP2 = (UTM_E2/(1-UTM_E2));        // e'^2


/**
 * Determine the correct UTM letter designator for the
 * given latitude
 *
 * @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com

 */
static inline char UTMLetterDesignator(double Lat)
{
        char LetterDesignator;

        if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
        else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
        else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
        else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
        else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
        else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
        else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
        else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
        else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
        else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
        else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
        else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
        else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
        else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
        else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
        else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
        else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
        else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
        else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
        else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
        // 'Z' is an error flag, the Latitude is outside the UTM limits
        else LetterDesignator = 'Z';
        return LetterDesignator;
}


/**
 * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
static inline void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           char* UTMZone)
{
        double a = WGS84_A;
        double eccSquared = UTM_E2;
        double k0 = UTM_K0;

        double LongOrigin;
        double eccPrimeSquared;
        double N, T, C, A, M;

        //Make sure the longitude is between -180.00 .. 179.9
        double LongTemp = (Long+180)-int((Long+180)/360)*360-180;

        double LatRad = Lat*RADIANS_PER_DEGREE;
        double LongRad = LongTemp*RADIANS_PER_DEGREE;
        double LongOriginRad;
        int    ZoneNumber;

        ZoneNumber = int((LongTemp + 180)/6) + 1;

        if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
                ZoneNumber = 32;
	//cout << "ZoneNumber: "<< ZoneNumber << endl;

        // Special zones for Svalbard
        if( Lat >= 72.0 && Lat < 84.0 )
        {
          if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
          else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
          else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
          else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
         }
        // +3 puts origin in middle of zone
        LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;
        LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;
	//cout << "Letter: "<< UTMLetterDesignator(Lat) << endl;
        //compute the UTM Zone from the latitude and longitude
        snprintf(UTMZone, 4, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));

        eccPrimeSquared = (eccSquared)/(1-eccSquared);

        N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
        T = tan(LatRad)*tan(LatRad);
        C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
        A = cos(LatRad)*(LongRad-LongOriginRad);

        M = a*((1        - eccSquared/4                - 3*eccSquared*eccSquared/64        - 5*eccSquared*eccSquared*eccSquared/256)*LatRad
                                - (3*eccSquared/8        + 3*eccSquared*eccSquared/32        + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
                                                                        + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
                                                                        - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

        UTMEasting = (double)(k0*N*(A+(1-T+C)*A*A*A/6
                                        + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
                                        + 500000.0);

        UTMNorthing = (double)(k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                                 + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
        if(Lat < 0)
                UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
}


void gpsCallBack(const sensor_msgs::NavSatFixConstPtr& gps)
{
	double northing, easting;
  	char zone;
	LLtoUTM(gps->latitude, gps->longitude,  northing, easting , &zone);
	global_position.x = easting;
	global_position.y = northing;
	global_position.z = gps->altitude;
}

int main(int argc, char** argv){
	ros::init(argc,argv, "Geoposition");
	ros::NodeHandle n;
	ros::Subscriber gps_sub = n.subscribe("fix",10, gpsCallBack);
	position_pub = n.advertise<geometry_msgs::Point> ("global_position", 1);
	ros::Rate loop_rate(10);
	while(n.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
