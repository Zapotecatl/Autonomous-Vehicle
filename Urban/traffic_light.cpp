#include "traffic_light.h"
#include "vehicle.h"
#include "sensor.h"
#include "measures.h"

STrafficLight **h_traffic_light;
STrafficLight **v_traffic_light;

STrafficSelfOrganizing **h_traffic_light_so;
STrafficSelfOrganizing **v_traffic_light_so;

int *h_regionTraffic_light[2];
int *v_regionTraffic_light[2];

int *h_regionTraffic_lightSO[2];
int *v_regionTraffic_lightSO[2];

float P;//Duration of a green light, i.e. half a period (T/2). To avoid stopping of vehicels, set this equal to half the length of the street or equal to a factor of half the length of the street.
float T;

int metodo_light;

double mod(double number1, double number2)
{
    return number1 - (floor (number1 / number2)) * number2;
}

void InializedTrafficLights(int metodo, float _P, int maxim_n, int maxim_m, int min_time, int max_time)
{

    metodo_light = metodo;

    //Inicializa semaforos ////////////////////////////////////////////////////
    InializedTrafficLightGreen(_P); //Onda verde
    //Auto Organizado
    InializedTrafficSelfOrganizing(maxim_n, maxim_m, min_time, max_time);
}


int allocateMemoryTrafficLight()
{

    int i;

    //reserved by horizontal
    h_traffic_light = new STrafficLight*[n_hor_streets];
    for (i = 0; i < n_hor_streets; i++)
        h_traffic_light[i] = new STrafficLight[m_ver_streets];

    //reserved by vertical
    v_traffic_light = new STrafficLight*[m_ver_streets];
    for (i = 0; i < m_ver_streets; i++)
        v_traffic_light[i] = new STrafficLight[n_hor_streets];

    h_regionTraffic_light[0] = new int[d_hor_street];
    h_regionTraffic_light[1] = new int[d_hor_street];

    v_regionTraffic_light[0] = new int[d_ver_street];
    v_regionTraffic_light[1] = new int[d_ver_street];

}

int freeTrafficLight()
{
    int i;

    for (i = 0; i < n_hor_streets; i++)
        delete [] h_traffic_light[i];
    delete [] h_traffic_light;

    for (i = 0; i < m_ver_streets; i++)
        delete [] v_traffic_light[i];
    delete [] v_traffic_light;


    delete [] h_regionTraffic_light[0];
    delete [] h_regionTraffic_light[1];

    delete []  v_regionTraffic_light[0];
    delete []  v_regionTraffic_light[1];

}

int allocateMemoryTrafficLightSO()
{

    int i, m, n;

    //reserved by horizontal
    h_traffic_light_so = new STrafficSelfOrganizing*[n_hor_streets];
    for (i = 0; i < n_hor_streets; i++)
        h_traffic_light_so[i] = new STrafficSelfOrganizing[m_ver_streets];

    //reserved by vertical
    v_traffic_light_so = new STrafficSelfOrganizing*[m_ver_streets];
    for (i = 0; i < m_ver_streets; i++)
        v_traffic_light_so[i] = new STrafficSelfOrganizing[n_hor_streets];

    h_regionTraffic_lightSO[0] = new int[d_hor_street];
    h_regionTraffic_lightSO[1] = new int[d_hor_street];

    v_regionTraffic_lightSO[0] = new int[d_ver_street];
    v_regionTraffic_lightSO[1] = new int[d_ver_street];



    return 0;

}


void freeTrafficLightSO()
{

    int i;

    for (i = 0; i < n_hor_streets; i++)
        delete [] h_traffic_light_so[i];
    delete [] h_traffic_light_so;

    for (i = 0; i < m_ver_streets; i++)
        delete [] v_traffic_light_so[i];
    delete [] v_traffic_light_so;


    delete [] h_regionTraffic_lightSO[0];
    delete [] h_regionTraffic_lightSO[1];

    delete [] v_regionTraffic_lightSO[0];
    delete [] v_regionTraffic_lightSO[1];

}


void regionTrafficLight() {

    //Create region for each traffic light
    int k, i, m, n;
    int reposition_cell_h;
    int reposition_cell_v;


    for (k = 0; k < 2; k++){
        for (m = 0; m < m_ver_streets; m++){

            reposition_cell_h = m * d_side_block + m;

            for (i = reposition_cell_h; i <= reposition_cell_h + d_side_block; i++){

                if (k == 0){
                    if ((m + 1) == m_ver_streets){
                        h_regionTraffic_light[k][i] = 0;
                    }
                    else{
                        h_regionTraffic_light[k][i] = m + 1;
                    }
                }
                else{
                    if (i == 0){
                        h_regionTraffic_light[k][i] = m_ver_streets - 1;
                    }
                    else {

                        if (i == reposition_cell_h)
                            h_regionTraffic_light[k][i] = m - 1;
                        else
                            h_regionTraffic_light[k][i] = m;
                    }
                }
                //printf("%d : %d - %d\n", k, i, h_regionTraffic_light[k][i]);
            }
        }
    }

    for (k = 0; k < 2; k++){
        for (n = 0; n < n_hor_streets; n++){

            reposition_cell_v = n * d_side_block + n;

            for (i = reposition_cell_v; i <= reposition_cell_v + d_side_block; i++){

                if (k == 0){
                    if ((n + 1) == n_hor_streets){
                        v_regionTraffic_light[k][i] = 0;
                    }
                    else{
                        v_regionTraffic_light[k][i] = n + 1;
                    }
                }
                else{
                    if (i == 0){
                        v_regionTraffic_light[k][i] = n_hor_streets - 1;
                    }
                    else {

                        if (i == reposition_cell_v)
                            v_regionTraffic_light[k][i] = n - 1;
                        else
                            v_regionTraffic_light[k][i] = n;
                    }
                }
                // printf("%d : %d - %d\n", k, i, v_regionTraffic_light[k][i]);
            }
        }
    }
}


void regionTrafficLightSO() {

    //Create region for each traffic light
    int k, i, m, n;
    int reposition_cell_h;
    int reposition_cell_v;


    for (k = 0; k < 2; k++){
        for (m = 0; m < m_ver_streets; m++){

            reposition_cell_h = m * d_side_block + m;

            for (i = reposition_cell_h; i <= reposition_cell_h + d_side_block; i++){

                if (k == 0){
                    if ((m + 1) == m_ver_streets){
                        h_regionTraffic_lightSO[k][i] = 0;
                    }
                    else{
                        h_regionTraffic_lightSO[k][i] = m + 1;
                    }
                }
                else{
                    if (i == 0){
                        h_regionTraffic_lightSO[k][i] = m_ver_streets - 1;
                    }
                    else {

                        if (i == reposition_cell_h)
                            h_regionTraffic_lightSO[k][i] = m - 1;
                        else
                            h_regionTraffic_lightSO[k][i] = m;
                    }
                }
                //printf("%d : %d - %d\n", k, i, h_regionTraffic_light[k][i]);
            }
        }
    }

    for (k = 0; k < 2; k++){
        for (n = 0; n < n_hor_streets; n++){

            reposition_cell_v = n * d_side_block + n;

            for (i = reposition_cell_v; i <= reposition_cell_v + d_side_block; i++){

                if (k == 0){
                    if ((n + 1) == n_hor_streets){
                        v_regionTraffic_lightSO[k][i] = 0;
                    }
                    else{
                        v_regionTraffic_lightSO[k][i] = n + 1;
                    }
                }
                else{
                    if (i == 0){
                        v_regionTraffic_lightSO[k][i] = n_hor_streets - 1;
                    }
                    else {

                        if (i == reposition_cell_v)
                            v_regionTraffic_lightSO[k][i] = n - 1;
                        else
                            v_regionTraffic_lightSO[k][i] = n;
                    }
                }
                // printf("%d : %d - %d\n", k, i, v_regionTraffic_light[k][i]);
            }
        }
    }
}



//Green wave///////////////////////////////////////////////////////////////


void InializedTrafficLightGreen(float _P)
{
    int n, m;
    int x;
    int y;
    int offset_t;

    allocateMemoryTrafficLight();

    P = _P + 1;//Porque en C empieza en cero las posiciones compenso con uno
    T = 2  * P;

    for (n = 0; n < n_hor_streets; n++){

        for (m = 0; m < m_ver_streets; m++){

            x = m * d_side_block + m;
            y = n * d_side_block + n;

            offset_t = floor(mod(x - y, T / 2.0) + 0.5);

            h_traffic_light[n][m].offset_t = offset_t;
            v_traffic_light[m][n].offset_t = offset_t;

            //printf("Offset %d: %d\n", m, h_traffic_light[m].offset_t);

            if (n % 2 == 0) {
                if (x == 0) {
                    h_traffic_light[n][m].position = d_hor_street - 1;
                    h_traffic_light[n][m].pos_intersection = 0;
                }
                else {
                    h_traffic_light[n][m].position = x - 1;
                    h_traffic_light[n][m].pos_intersection = x;
                }
            }
            else {
                h_traffic_light[n][m].position = x + 1;
                h_traffic_light[n][m].pos_intersection = x;
            }

            if (m % 2 == 0) {
                if (y == 0){
                    v_traffic_light[m][n].position = d_ver_street - 1;
                    v_traffic_light[m][n].pos_intersection = 0;
                }
                else {
                    v_traffic_light[m][n].position = y - 1;
                    v_traffic_light[m][n].pos_intersection = y;
                }
            }
            else {
                v_traffic_light[m][n].position = y + 1;
                v_traffic_light[m][n].pos_intersection = y;
            }


            if (floor(mod(x - y, T) + 0.5) >= (T / 2.0)) {

                h_traffic_light[n][m].light = 0;//Red
                v_traffic_light[m][n].light = 1;// Green

                h_traffic_light[n][m].time = P;
                v_traffic_light[m][n].time = v_traffic_light[m][n].offset_t;
                //printf("Red Offset %d: %d\n", m, h_traffic_light[n][m].offset_t);
            }
            else {
                h_traffic_light[n][m].light = 1;//Green
                v_traffic_light[m][n].light = 0;//Red

                h_traffic_light[n][m].time = h_traffic_light[n][m].offset_t;
                v_traffic_light[m][n].time = P;
               // printf("Green Offset %d: %d\n", m, h_traffic_light[n][m].offset_t);
            }
        }
    }

    regionTrafficLight();
}

void TrafficLightGreenWave(int n, int m)
{

    if (GetValueTrafficLight('H', n, m) == 1){

        if (h_traffic_light[n][m].time < P)
            h_traffic_light[n][m].time++;
        else{
            SwitchTrafficLight(n, m);
        }
    }
    else {
        if (v_traffic_light[m][n].time < P)
            v_traffic_light[m][n].time++;
        else{
            SwitchTrafficLight(n, m);
        }
    }
}

void SwitchTrafficLight(int n, int m)
{

    if (n < 0)
        n = 0;
    else if (n >= n_hor_streets)
        n = n_hor_streets - 1;

    if (m < 0)
        m = 0;
    else if (m >= m_ver_streets)
        m = m_ver_streets - 1;


         if (h_traffic_light[n][m].light == 0){
            h_traffic_light[n][m].light = 1;//Green
            v_traffic_light[m][n].light = 0;//Red

            h_traffic_light[n][m].time = 0;
            v_traffic_light[m][n].time = P;
        }
        else {
            h_traffic_light[n][m].light = 0;//Red
            v_traffic_light[m][n].light = 1;//Green

            v_traffic_light[m][n].time = 0;
            h_traffic_light[n][m].time = P;
        }
}

int GetValueTrafficLight(char type_street, int n, int m)
{

    if (type_street == 'H') {
        return h_traffic_light[n][m].light;
    }
    else if(type_street == 'V') {
        return v_traffic_light[m][n].light;
    }

    return -1;
}


int GetPositionTrafficLight(char type_street, int n, int m)
{
    int value;

    if (type_street == 'H')
       value = h_traffic_light[n][m].position;
    else
       value = v_traffic_light[m][n].position;

    return value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void InializedTrafficSelfOrganizing(int maxim_n, int maxim_m, int min_time, int max_time)
{
    int n, m;
    int x, y;


    if (maxim_n < 0)
        maxim_n = 0;

    if (maxim_m < 0)
        maxim_m = 0;

    if (max_time < 0)
        max_time = 0;

    if (min_time < 0)
        min_time = 0;

    allocateMemoryTrafficLightSO();

    for (n = 0; n < n_hor_streets; n++) {
        for (m = 0; m < m_ver_streets; m++) {

            h_traffic_light_so[n][m].light = 0;
            v_traffic_light_so[m][n].light = 1;

            h_traffic_light_so[n][m].time_u = 0;
            v_traffic_light_so[m][n].time_u = 0;

            h_traffic_light_so[n][m].n_sum_veh = 0;
            v_traffic_light_so[m][n].n_sum_veh = 0;

            h_traffic_light_so[n][m].n_vehicles = 0;
            v_traffic_light_so[m][n].n_vehicles = 0;

            h_traffic_light_so[n][m].m_vehicles = 0;
            v_traffic_light_so[m][n].m_vehicles = 0;

            h_traffic_light_so[n][m].vehicle_stop = false;
            v_traffic_light_so[m][n].vehicle_stop = false;

            h_traffic_light_so[n][m].changed = false;
            v_traffic_light_so[m][n].changed = false;

            h_traffic_light_so[n][m].min_time = min_time;
            v_traffic_light_so[m][n].min_time = min_time;

            h_traffic_light_so[n][m].max_time = max_time;
            v_traffic_light_so[m][n].max_time = max_time;

            h_traffic_light_so[n][m].maxim_n = maxim_n;
            v_traffic_light_so[m][n].maxim_n = maxim_n;

            //qDebug() << "Tosssss" << maxim_m << maxim_n;

            h_traffic_light_so[n][m].maxim_m = maxim_m;
            v_traffic_light_so[m][n].maxim_m = maxim_m;


            x = m * d_side_block + m;
            y = n * d_side_block + n;

            if (n % 2 == 0) {
                if (x == 0) {
                    h_traffic_light_so[n][m].position = d_hor_street - 1;
                    h_traffic_light_so[n][m].pos_intersection = 0;
                }
                else {
                    h_traffic_light_so[n][m].position = x - 1;
                    h_traffic_light_so[n][m].pos_intersection = x;
                }
            }
            else {
                h_traffic_light_so[n][m].position = x + 1;
                h_traffic_light_so[n][m].pos_intersection = x;
            }

            if (m % 2 == 0) {
                if (y == 0){
                    v_traffic_light_so[m][n].position = d_ver_street - 1;
                    v_traffic_light_so[m][n].pos_intersection = 0;
                }
                else {
                    v_traffic_light_so[m][n].position = y - 1;
                    v_traffic_light_so[m][n].pos_intersection = y;
                }
            }
            else {
                v_traffic_light_so[m][n].position = y + 1;
                v_traffic_light_so[m][n].pos_intersection = y;
            }
        }
    }


     regionTrafficLightSO();

}


void RunTrafficLight(int n, int m)
{
    if (metodo_light == 1)
        TrafficLightGreenWave(n, m);
    else {

        if (metodo_sensado == 1)
            SensingSelfOrganizing(n, m);
        else
            SensingSelfOrganizingSensor(n, m);

        TrafficLightSelfOrganizing(n, m);
    }
}

void TrafficLightSelfOrganizing(int n, int m)
{

    //    qDebug() << h_traffic_light_so[n][m].vehicle_stop;
    //    qDebug() << h_traffic_light_so[n][m].n_vehicles;
    //    qDebug() << h_traffic_light_so[n][m].m_vehicles;

    h_traffic_light_so[n][m].changed = false;
    v_traffic_light_so[m][n].changed = false;

    h_traffic_light_so[n][m].time_u++;
    v_traffic_light_so[m][n].time_u++;

    if (h_traffic_light_so[n][m].light == 1){

        if (h_traffic_light_so[n][m].vehicle_stop == true){
            if (v_traffic_light_so[m][n].vehicle_stop == true)
                SwitchBothRedTrafficLightSO(n, m); //regla 6
            else
                SwitchTrafficLightSO(n, m); // regla 5
        }
        else {

            if (v_traffic_light_so[m][n].vehicle_stop == false) {

                if (v_traffic_light_so[m][n].n_vehicles >= 1  && h_traffic_light_so[n][m].n_vehicles == 0) {
                    SwitchTrafficLightSO(n, m); //regla 4
                }
                else if (!(h_traffic_light_so[n][m].m_vehicles > 0 && h_traffic_light_so[n][m].m_vehicles < h_traffic_light_so[n][m].maxim_m)) {
                    if (h_traffic_light_so[n][m].time_u >= h_traffic_light_so[n][m].min_time)
                        if (v_traffic_light_so[m][n].n_sum_veh >= v_traffic_light_so[m][n].maxim_n || v_traffic_light_so[m][n].time_u >= v_traffic_light_so[m][n].max_time) {
                            //      if (v_traffic_light_so[m][n].n_vehicles >= v_traffic_light_so[m][n].maxim_n) {

                            // qDebug () << "Cambio por cantidad de automoviles H";
                            SwitchTrafficLightSO(n, m);
                        }
                }
            }
        }
    }
    else if (v_traffic_light_so[m][n].light == 1) {


        if (v_traffic_light_so[m][n].vehicle_stop == true) {

            if (h_traffic_light_so[n][m].vehicle_stop == true)
                SwitchBothRedTrafficLightSO(n, m); //regla 6
            else
                SwitchTrafficLightSO(n, m); // regla 5
        }
        else {

            if (h_traffic_light_so[n][m].vehicle_stop == false) {

                if (h_traffic_light_so[n][m].n_vehicles >= 1  && v_traffic_light_so[m][n].n_vehicles == 0) {
                    SwitchTrafficLightSO(n, m); //regla 4
                }
                else if (!(v_traffic_light_so[m][n].m_vehicles > 0 && v_traffic_light_so[m][n].m_vehicles < v_traffic_light_so[m][n].maxim_m)) {
                    if (v_traffic_light_so[m][n].time_u >= v_traffic_light_so[m][n].min_time)
                        if (h_traffic_light_so[n][m].n_sum_veh >= h_traffic_light_so[n][m].maxim_n || h_traffic_light_so[n][m].time_u >= h_traffic_light_so[n][m].max_time) {
                            //    if (h_traffic_light_so[n][m].n_vehicles >= h_traffic_light_so[n][m].maxim_n) {

                            // qDebug () << "Cambio por cantidad de automoviles V";
                            SwitchTrafficLightSO(n, m);
                        }
                }
            }
        }
    }
    else{ //implicito que ambos estan en rojo
        if (h_traffic_light_so[n][m].vehicle_stop == false)
            RestoreSingleGreen('H', n, m);
        else if (v_traffic_light_so[m][n].vehicle_stop == false)
            RestoreSingleGreen('V', n, m);
    }

}



int GetRegionTrafficLight(char type_street, int x, int direction)
{

    int value;

    if (type_street == 'H') {

        if (direction == 'R')
            value = h_regionTraffic_light[0][x];
        else
            value = h_regionTraffic_light[1][x];
    }
    else {

        if (direction == 'R')
            value = v_regionTraffic_light[0][x];
        else
            value = v_regionTraffic_light[1][x];
    }

  return value;

}

int GetPositionIntersectionTrafficLight(char type_street, int n, int m)
{
    int pos;

    if (type_street == 'H')
        pos = h_traffic_light[n][m].pos_intersection;
    else
        pos = v_traffic_light[m][n].pos_intersection;

    return pos;
}



int GetRegionTrafficLightSO(char type_street, int x, int direction)
{

    int value;

    if (type_street == 'H') {

        if (direction == 'R')
            value = h_regionTraffic_lightSO[0][x];
        else
            value = h_regionTraffic_lightSO[1][x];
    }
    else {

        if (direction == 'R')
            value = v_regionTraffic_lightSO[0][x];
        else
            value = v_regionTraffic_lightSO[1][x];
    }

  return value;

}


void SwitchTrafficLightSO(int n, int m)
{

    if (n < 0)
        n = 0;
    else if (n >= n_hor_streets)
        n = n_hor_streets - 1;

    if (m < 0)
        m = 0;
    else if (m >= m_ver_streets)
        m = m_ver_streets - 1;


    h_traffic_light_so[n][m].time_u = 0;
    v_traffic_light_so[m][n].time_u = 0;

    if (h_traffic_light_so[n][m].light == 0){

        h_traffic_light_so[n][m].light = 1;//Green
        v_traffic_light_so[m][n].light = 0;//Red

        h_traffic_light_so[n][m].n_sum_veh = 0;

    }
    else {

        h_traffic_light_so[n][m].light = 0;//Red
        v_traffic_light_so[m][n].light = 1;//Green

        v_traffic_light_so[m][n].n_sum_veh = 0;
    }


    h_traffic_light_so[n][m].changed = true;
    v_traffic_light_so[m][n].changed = true;

}

void SwitchBothRedTrafficLightSO(int n, int m)
{
        h_traffic_light_so[n][m].time_u = 0;
        v_traffic_light_so[m][n].time_u = 0;

        h_traffic_light_so[n][m].light = 0;//Red
        v_traffic_light_so[m][n].light = 0;//Red
}

int GetValueTrafficLightSO(char type_street, int n, int m)
{

    if (n < 0 || m < 0)
        return -1;//no definido

    if (n >= n_hor_streets)
        n = 0;

    if (m >= m_ver_streets)
        m = 0;

    if (type_street == 'H')
        return h_traffic_light_so[n][m].light;
    else
        return v_traffic_light_so[m][n].light;

    return -1;
}

int GetPositionIntersectionTrafficLightSO(char type_street, int n, int m)
{
    int pos;

    if (type_street == 'H')
        pos = h_traffic_light_so[n][m].pos_intersection;
    else
        pos = v_traffic_light_so[m][n].pos_intersection;

    return pos;
}

int GetPositionTrafficLightSO(char type_street, int n, int m)
{
    int pos;

    if (type_street == 'H')
        pos = h_traffic_light_so[n][m].position;
    else
        pos = v_traffic_light_so[m][n].position;

    return pos;
}

void RestoreSingleGreen(char type_street, int n, int m)
{

    h_traffic_light_so[n][m].time_u = 0;
    v_traffic_light_so[m][n].time_u = 0;

    if (type_street == 'H'){

        h_traffic_light_so[n][m].light = 1;//Green
        v_traffic_light_so[m][n].light = 0;//Red

    }
    else{

        h_traffic_light_so[n][m].light = 0;//Red
        v_traffic_light_so[m][n].light = 1;//Green
    }
}

