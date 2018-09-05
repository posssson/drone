#include <stdio.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"

#include <wiringPi.h>
#include "MPU9250.h"

#include "mathlib/mathlib_onera_v3.h"
#include "conversion/rotation.h"
#include <iostream>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
using namespace std ;
#define OK 1
#define CONSTANTS_ONE_G  9.80665
#define ERROR -1
int detect_orientation (MPU9250 * sub_sensor_combined) ;
int do_accel_calibration_measurements (double accel_offs_MPU[3] , double accel_T_MPU[3][3] , MPU9250 * mpu ) ;
int read_accelerometer_avg (MPU9250 *sensor_combined_sub , double accel_avg[3] , int samples_num) ;
int calculate_calibration_values (double accel_ref[6][3] , double accel_T[3][3] , double accel_offs[3] , float g) ;
int mat_invert3 (float src[3][3] , float dst[3][3]) ;
int sphere_fit_least_squares2 (const float *hx , const float *hy , const float *hz ,
                               int nt , int nitmax , float delta , float *sphere_x , float *sphere_y , float *sphere_z ,
                               float *sphere_radius) ;
float ax, ay, az,mx,my,mz;
float gx , gy, gz;
struct accel_scale_xsens
{	//RF_ONERA
    double x_offset ;
    double x_scale ;
    double y_offset ;
    double y_scale ;
    double z_offset ;
    double z_scale ;
} ;
MPU9250 mpu(0x68);
int main (int argc , char** argv)
{

    int i ;
    std::cout << "Quelle calibration voulez vous faire? \n\t 1 - Accéléromètre MPU9250 et LSM9DS1 \n\t 2 - Gyromètre MPU9250 et LSM9DS1\n\t 3 - Magnétomètre" << std::endl ;

    cin >>i ;
    //i = 3 ;
  

    std::cout << "Selected: MPU9250" << std::endl ;
  mpu.begin(ACCEL_RANGE_8G,GYRO_RANGE_1000DPS);


    if ( i == 1 )
    {


        // param_t board_rotation_h = param_find ("SENS_BOARD_ROT");
        int32_t board_rotation_int = 0 ; // paramètre de rotation du raspberry
        enum Rotation board_rotation_id = ( enum Rotation )board_rotation_int ;
        math::Matrix<3 , 3> board_rotation ;



        accel_scale_xsens accel_scale_MPU ;
        accel_scale_MPU.x_offset = 0 ;
        accel_scale_MPU.x_scale = 1 ;
        accel_scale_MPU.y_offset = 0 ;
        accel_scale_MPU.y_scale = 1 ;
        accel_scale_MPU.z_offset = 0 ;
        accel_scale_MPU.z_scale = 1 ;
        double accel_offs_MPU[3] ;
        double accel_T_MPU[3][3] ;

      
        do_accel_calibration_measurements (accel_offs_MPU ,  accel_T_MPU , &mpu ) ;
        get_rot_matrix (board_rotation_id , &board_rotation) ;
        math::Matrix<3 , 3> board_rotation_t = board_rotation.transposed () ;
        math::Vector<3> accel_offs_vec_MPU (accel_offs_MPU) ;
        math::Vector<3> accel_offs_rotated_MPU = board_rotation_t * accel_offs_vec_MPU ;
        math::Matrix<3 , 3> accel_T_mat_MPU (accel_T_MPU) ;
        math::Matrix<3 , 3> accel_T_rotated_MPU = board_rotation_t * accel_T_mat_MPU * board_rotation ;
        accel_scale_MPU.x_offset = accel_offs_rotated_MPU (1) ;
        accel_scale_MPU.x_scale = accel_T_rotated_MPU (1 , 1) ;
        accel_scale_MPU.y_offset = accel_offs_rotated_MPU (2) ;
        accel_scale_MPU.y_scale = accel_T_rotated_MPU (2 , 2) ;
        accel_scale_MPU.z_offset = accel_offs_rotated_MPU (3) ;
        accel_scale_MPU.z_scale = accel_T_rotated_MPU (3 , 3) ;




        // Après modification de l'ordre MPU
        cout << "accx_offset_MPU = " << accel_scale_MPU.x_offset << endl ;
        cout << "accx_scale_MPU = " <<  accel_scale_MPU.x_scale << endl ;
        cout << "accy_offset_MPU = " << accel_scale_MPU.y_offset << endl ;
        cout << "accy_scale_MPU = " <<  accel_scale_MPU.y_scale << endl ;
        cout << "accz_offset_MPU = " << accel_scale_MPU.z_offset  << endl ;
        cout << "accz_scale_MPU = " <<  accel_scale_MPU.z_scale  << endl ;

 
        float accel [6] = {
                            accel_scale_MPU.x_offset ,
                            accel_scale_MPU.x_scale ,
                            accel_scale_MPU.y_offset ,
                            accel_scale_MPU.y_scale ,
                            accel_scale_MPU.z_offset ,
                            accel_scale_MPU.z_scale ,

        } ;

      /*  FILE *pFile ;
        pFile = fopen ("/media/clef/accel_offset_scale.txt" , "w") ;
        fflush (stdin) ;
        fflush (stdout) ;
        for ( int i = 0 ; i < 12 ; i++ )
        {
            fprintf (pFile , "%f \n" , accel[i]) ;
        }

        fclose (pFile) ; // je referme le fichier*/
    }
    if ( i == 2 )
    {
        cout << "On ne bouge pas!" << endl ;
        float gx_mpu , gy_mpu , gz_mpu ;

        float somme_x_mpu = 0 , somme_y_mpu = 0 , somme_z_mpu = 0 ;

        float moyenne_x_mpu , moyenne_y_mpu , moyenne_z_mpu ;


        int compte ;
        int count = 0 ;
        for ( compte = 1 ; compte < 5000 ; compte++ )
        {
            usleep (10) ;
			mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz,&mx, &my, &mz);
			somme_x_mpu += gx ;
            somme_y_mpu += gy ;
            somme_z_mpu += gz ;

            count ++ ;
        }
        moyenne_x_mpu = somme_x_mpu / count ;
        moyenne_y_mpu = somme_y_mpu / count ;
        moyenne_z_mpu = somme_z_mpu / count ;


        cout << "gyrox_offset_MPU = " << moyenne_x_mpu << endl ;
        cout << "gyroy_offset_MPU = " << moyenne_y_mpu << endl ;
        cout << "gyroz_offset_MPU = " << moyenne_z_mpu  << endl ;

   

        float gyrooffset [3] = {
                                moyenne_x_mpu ,
                                moyenne_y_mpu ,
                                moyenne_z_mpu ,
           
        } ;

      /*  FILE *pFile ;
        pFile = fopen ("/media/clef/gyro_offset.txt" , "w") ;
        fflush (stdin) ;
        fflush (stdout) ;
        for ( int i = 0 ; i < 6 ; i++ )
        {
            fprintf (pFile , "%f \n" , gyrooffset[i]) ;
        }

        fclose (pFile) ; // je referme le fichier*/

    }
    if ( i == 3 )
    {
        int nombre = 5000 ;
        float mx_MPU[nombre] , my_MPU[nombre] , mz_MPU[nombre] ;
     
        float mx_MPUc[nombre] , my_MPUc[nombre] , mz_MPUc[nombre] ;
   
        float sphere_x_mpu , sphere_y_mpu , sphere_z_mpu , sphere_radius_mpu ;
      
        cout << "rotate in a figure 8 around all axis" << endl ;
        int compte = 0 ;
     /*   FILE *pFile ;
        pFile = fopen ("/media/clef/vecteurmpu.txt" , "w") ;
        FILE *pFilec ;
        pFilec = fopen ("/media/clef/vecteurlsm.txt" , "w") ;
        fflush (stdin) ;
        fflush (stdout) ;*/

        for ( compte = 0 ; compte < nombre + 10 ; compte ++ )
        {



            usleep (1000) ;
			mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz,&mx_MPU[compte], &my_MPU[compte], &mz_MPU[compte]);

              }

        for ( compte = 0 ; compte < nombre ; compte ++ )
        {
            mx_MPUc[compte] = mx_MPU[compte + 10];// / 1000 ;
            my_MPUc[compte] = my_MPU[compte + 10];// / 1000 ;
            mz_MPUc[compte] = mz_MPU[compte + 10];// / 1000 ;
        
        //    fprintf (pFile , "\t%f \t%f \t%f \n " , mx_MPUc[compte] , my_MPUc[compte] , mz_MPUc[compte]) ;
              }

      /*  fclose (pFile) ;
        fclose (pFilec) ;*/
        cout << "Fin for" << endl ;
          if ( sphere_fit_least_squares2 (mx_MPUc , my_MPUc , mz_MPUc , nombre , 100 , 0 , &sphere_x_mpu , &sphere_y_mpu , &sphere_z_mpu , &sphere_radius_mpu) == 0 )
          {
              cout << "Succes MPU" << endl ;
          }
       
        cout << "Fin de sphere fit least square" << endl ;
        cout << sphere_x_mpu << " " << sphere_y_mpu << " " << sphere_z_mpu << " " << endl ;
       /* FILE *pFilecc ;
        pFilecc = fopen ("/media/clef/magn_offset.txt" , "w") ;
        fflush (stdin) ;
        fflush (stdout) ;

        fprintf (pFilecc , "MPUoffx %f \t MPUoffy %f \tMPUoffx %f \t\n" , sphere_x_mpu , sphere_y_mpu , sphere_z_mpu) ;
        fprintf (pFilecc , "LSMoffx %f \t LSMoffy %f \tLSMoffx %f \t\n" , sphere_x_lsm , sphere_y_lsm , sphere_z_lsm) ;


        fclose (pFilecc) ; // je referme le fichier*/

    }


}

int sphere_fit_least_squares2 (const float *hx , const float *hy , const float *hz ,
                               int nt , int nitmax , float delta , float *sphere_x , float *sphere_y , float *sphere_z ,
                               float *sphere_radius)
{


    float   *hxc , *hyc , *hzc ;
    float   *Y ;
    //    float   Bx , By , Bz , Br ;

    float   bx , by , bz , ray ;
    int i , k ;
    float dr , dr2 , dr1 ;

    math::Vector<4> Ym ;
    math::Matrixs<4> M , M1 ;
    math::Vector<4> P , dP ;


    hxc = ( float* ) malloc (sizeof (float ) * nt) ;
    hyc = ( float* ) malloc (sizeof (float ) * nt) ;
    hzc = ( float* ) malloc (sizeof (float ) * nt) ;
    Y   = ( float* ) malloc (sizeof (float ) * nt) ;

    P (1) = 0.0 ;
    P (2) = 0.0 ;
    P (3) = 0.0 ;
    P (4) = 1.0 ;

    for ( k = 0 ; k < nitmax ; k++ )
    {

        dr2 = 0.0 ;
        for ( i = 0 ; i < nt ; i++ )
        {
            bx = P (1) ;
            by = P (2) ;
            bz = P (3) ;
            ray = P (4) ;

            hxc[i] = hx[i] - bx ;
            hyc[i] = hy[i] - by ;
            hzc[i] = hz[i] - bz ;

            Y[i] = hxc[i] * hxc[i] + hyc[i] * hyc[i] + hzc[i] * hzc[i] - ray*ray ;

            dr2 = dr2 + Y[i] * Y[i] ;
        }

        dr = sqrt (dr2) ;

        if ( k > 0 )
            if ( fabs (dr - dr1) < delta )
                break ;

        dr1 = dr ;

        //        printf("calmag dr %10.3f\n", dr);

        Ym.zero () ;
        M.zero () ;

        for ( i = 0 ; i < nt ; i++ )
        {
            Ym (1) = Ym (1) + 2.0 * hxc[i] * Y[i] ;
            Ym (2) = Ym (2) + 2.0 * hyc[i] * Y[i] ;
            Ym (3) = Ym (3) + 2.0 * hzc[i] * Y[i] ;
            Ym (4) = Ym (4) + 2.0 * ray * Y[i] ;

            M (1 , 1) = M (1 , 1) + 4.0 * hxc[i] * hxc[i] ;
            M (1 , 2) = M (1 , 2) + 4.0 * hxc[i] * hyc[i] ;
            M (1 , 3) = M (1 , 3) + 4.0 * hxc[i] * hzc[i] ;
            M (1 , 4) = M (1 , 4) + 4.0 * hxc[i] * ray ;
            M (2 , 2) = M (2 , 2) + 4.0 * hyc[i] * hyc[i] ;
            M (2 , 3) = M (2 , 3) + 4.0 * hyc[i] * hzc[i] ;
            M (2 , 4) = M (2 , 4) + 4.0 * hyc[i] * ray ;
            M (3 , 3) = M (3 , 3) + 4.0 * hzc[i] * hzc[i] ;
            M (3 , 4) = M (3 , 4) + 4.0 * hzc[i] * ray ;
            M (4 , 4) = M (4 , 4) + 4.0 * ray*ray ;
        }
        //       Ym.print("Ym");
        //std::printf ("Sphere fit\n") ;
        //M.print ("M") ;

        M1 = M.inversc () ;

        // M1.print ("M1") ;

        dP = M1*Ym ;

        P = P + dP ;
        //printf ("calmag2 A B C %10.5f %10.5f %10.5f\n" , P (1) , P (2) , P (3)) ;

    }

    *sphere_x = P (1) ;
    *sphere_y = P (2) ;
    *sphere_z = P (3) ;
    *sphere_radius = P (4) ;

    return 0 ;
}

int do_accel_calibration_measurements (double accel_offs_MPU[3]  , double accel_T_MPU[3][3] ,  MPU9250 * mpu )
{
    const int samples_num = 1000 ;	//RF_ONERA
    double accel_ref_MPU[6][3] ;
    bool data_collected[7] = { false , false , false , false , false , false , true } ;
    const char *orientation_strs[7] = { "x+" , "x-" , "y+" , "y-" , "z+" , "z-" , "INCONNUE" } ;
    int res = OK ;



    unsigned done_count = 0 ;


    while ( true )
    {
        bool done = true ;
        //        unsigned old_done_count = done_count;
        done_count = 0 ;

        for ( int i = 0 ; i < 6 ; i++ )
        {
            if ( data_collected[i] )
            {
                done_count++ ;

            }
            else
            {
                done = false ;
            }
        }

        if ( done )
            break ;
        printf ("directions left: %s%s%s%s%s%s" ,
                ( !data_collected[0] ) ? "x+ " : "" ,
                ( !data_collected[1] ) ? "x- " : "" ,
                ( !data_collected[2] ) ? "y+ " : "" ,
                ( !data_collected[3] ) ? "y- " : "" ,
                ( !data_collected[4] ) ? "z+ " : "" ,
                ( !data_collected[5] ) ? "z- " : "") ;
        cout << " " << endl ;
        sleep (5) ;
        int orient = detect_orientation (mpu) ; // 
        cout <<  "Orientation = " << orientation_strs[orient] << endl ;
        if ( orient < 0 )
        {
            res = ERROR ;
            break ;
        }
        if ( orient == 6 )
        {
            cout <<  "Je ne reconnais pas la position... Try again.." << endl ;
        }
        if ( data_collected[orient] )
        {
            cout << orientation_strs[orient] << " done, rotate to a different axis" << endl ;

        }
        else
        {
            read_accelerometer_avg (mpu , &( accel_ref_MPU[orient][0] ) , samples_num) ;  // On récupere les moyennes de l'orientation qu'on a choisit sur 1000 itérations.
             data_collected[orient] = true ;
            cout << orientation_strs[orient] << " OK!" << endl ;
        }

    }
    if ( res == OK )
    {
        /* calculate offsets and transform matrix */
        cout << "\nCalcul Offset et Scale\n" << endl ;
        float gravite = ( float ) CONSTANTS_ONE_G ;
        res = calculate_calibration_values (accel_ref_MPU , accel_T_MPU , accel_offs_MPU , gravite) ;
      
        if ( res != OK )
        {
            cout << "ERROR: calibration values calculation error" << endl ;
        }
    }
    return res ;
}

int detect_orientation (MPU9250 * sub_sensor_combined)
{
    // cout << "Detection orientation" << endl;

    float accel_ema[3] = { 0.0 , 0.0 , 0.0 } ;
    float accel_err_thr = 2.0 ;
	mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz,&mx, &my, &mz);

    accel_ema[0] = ax ;
    accel_ema[1] = ay ;
    accel_ema[2] = az ;
    if ( fabs (accel_ema[0] - float(CONSTANTS_ONE_G )) < accel_err_thr &&
         fabs (accel_ema[1]) < accel_err_thr &&
         fabs (accel_ema[2]) < accel_err_thr )

        return 0 ;	// [ g, 0, 0 ]

    if ( fabs (accel_ema[0] + float(CONSTANTS_ONE_G )) < accel_err_thr &&
         fabs (accel_ema[1]) < accel_err_thr &&
         fabs (accel_ema[2]) < accel_err_thr )

        return 1 ;	// [ -g, 0, 0 ]

    if ( fabs (accel_ema[0]) < accel_err_thr &&
         fabs (accel_ema[1] - float(CONSTANTS_ONE_G )) < accel_err_thr &&
         fabs (accel_ema[2]) < accel_err_thr )

        return 2 ;	// [ 0, g, 0 ]

    if ( fabs (accel_ema[0]) < accel_err_thr &&
         fabs (accel_ema[1] + float(CONSTANTS_ONE_G )) < accel_err_thr &&
         fabs (accel_ema[2]) < accel_err_thr )
        return 3 ;	// [ 0, -g, 0 ]

    if ( fabs (accel_ema[0]) < accel_err_thr &&
         fabs (accel_ema[1]) < accel_err_thr &&
         fabs (accel_ema[2] - float(CONSTANTS_ONE_G )) < accel_err_thr )
    {

        return 4 ;	// [ 0, 0, g ]
    }

    if ( fabs (accel_ema[0]) < accel_err_thr &&
         fabs (accel_ema[1]) < accel_err_thr &&
         fabs (accel_ema[2] + float(CONSTANTS_ONE_G )) < accel_err_thr )
    {
        return 5 ;	// [ 0, 0, -g ]
    }
    return 6 ;  // On trouve pas la position
}

int read_accelerometer_avg (MPU9250 *sub_sensor_combined , double accel_avg[3] , int samples_num)
{
    cout << " \n\n On bouge plus!\n\n" << endl ;
    sleep (1) ;

    int count = 0 ;
    float accel_sum[3] = { 0.0f , 0.0f , 0.0f } ;

    while ( count < samples_num )
    {

        usleep (1000) ;
        float accel_ema[3] = { 0.0f , 0.0f , 0.0f } ;

		mpu.getMotion9(&accel_ema[0], &accel_ema[1], &accel_ema[2], &gx, &gy, &gz,&mx, &my, &mz);
        for ( int i = 0 ; i < 3 ; i++ )
            accel_sum[i] += accel_ema[i] ; // On fait la somme des trois valeurs.

        count++ ;

    }

    for ( int i = 0 ; i < 3 ; i++ )
    {
        accel_avg[i] = (double)accel_sum[i] / count ; // On fait la moyenne sur les messures.
    }
    cout << "Calcul moyenne x = " << accel_avg[0] << endl ;
    cout << "Calcul moyenne y = " << accel_avg[1] << endl ;
    cout << "Calcul moyenne z = " << accel_avg[2] << endl ;
    cout << "" << endl ;
    return OK ;
}

int mat_invert3 (float src[3][3] , float dst[3][3])
{
    float det = src[0][0] * ( src[1][1] * src[2][2] - src[1][2] * src[2][1] ) -
            src[0][1] * ( src[1][0] * src[2][2] - src[1][2] * src[2][0] ) +
            src[0][2] * ( src[1][0] * src[2][1] - src[1][1] * src[2][0] ) ;

    if ( det == 0.0 )
        return ERROR ;	// Singular matrix

    dst[0][0] = ( src[1][1] * src[2][2] - src[1][2] * src[2][1] ) / det ;
    dst[1][0] = ( src[1][2] * src[2][0] - src[1][0] * src[2][2] ) / det ;
    dst[2][0] = ( src[1][0] * src[2][1] - src[1][1] * src[2][0] ) / det ;
    dst[0][1] = ( src[0][2] * src[2][1] - src[0][1] * src[2][2] ) / det ;
    dst[1][1] = ( src[0][0] * src[2][2] - src[0][2] * src[2][0] ) / det ;
    dst[2][1] = ( src[0][1] * src[2][0] - src[0][0] * src[2][1] ) / det ;
    dst[0][2] = ( src[0][1] * src[1][2] - src[0][2] * src[1][1] ) / det ;
    dst[1][2] = ( src[0][2] * src[1][0] - src[0][0] * src[1][2] ) / det ;
    dst[2][2] = ( src[0][0] * src[1][1] - src[0][1] * src[1][0] ) / det ;

    return OK ;
}

int calculate_calibration_values (double accel_ref[6][3] , double accel_T[3][3] , double accel_offs[3] , float g)
{
    /* calculate offsets */
    for ( int i = 0 ; i < 3 ; i++ )
    {
        accel_offs[i] = ( accel_ref[i * 2][i] + accel_ref[i * 2 + 1][i] ) / 2 ;
    }

    /* fill matrix A for linear equations system*/
    float mat_A[3][3] ;
    memset (mat_A , 0 , sizeof (mat_A )) ;

    for ( int i = 0 ; i < 3 ; i++ )
    {
        for ( int j = 0 ; j < 3 ; j++ )
        {
            float a = accel_ref[i * 2][j] - accel_offs[j] ;
            mat_A[i][j] = a ;
        }
    }

    /* calculate inverse matrix for A */
    float mat_A_inv[3][3] ;

    if ( mat_invert3 (mat_A , mat_A_inv) != OK )
        return ERROR ;

    /* copy results to accel_T */
    for ( int i = 0 ; i < 3 ; i++ )
    {
        for ( int j = 0 ; j < 3 ; j++ )
        {
            /* simplify matrices mult because b has only one non-zero element == g at index i */
            accel_T[j][i] = mat_A_inv[j][i] * (double)g ;
        }
    }

    return OK ;
}
