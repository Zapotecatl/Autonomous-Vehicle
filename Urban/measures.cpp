
#include "measures.h"
#include "vehicle.h"

float velocity;
float flux;

float v_optim;
float j_optim;
float j_max = 1.0 / 4.0;

float velocity_total;
float flux_total;

//Measures////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float vOptima(float density)
{

    //Es mejor graficar desde R graphics porque se pierden puntos criticos
    if (density <= j_max)
        return 1.0;
    else if (density >= (1.0 - j_max))
        return (1.0 - density) / density;
    else if (density > j_max && density < (1.0 - j_max))
        return j_max / density;

    return 0.0;
}

float jOptima(float density)
{

    //Es mejor graficar desde R graphics porque se pierden puntos criticos
    if (density <= j_max)
        return density;
    else if (density >= (1.0 - j_max))
        return (1.0 - density);
    else if (density > j_max && density < (1.0 - j_max))
        return j_max;

    return 0.0;
}


void Velocity()
{

    int i;
    int v_total;

    v_total = 0;
    for (i = 0; i < size_vehicles; i++) {
        v_total+= GetVelocityVehicle(i);
    }

    if (size_vehicles != 0)
      velocity = ((float) v_total / size_vehicles);
    else
      velocity = 0;

    //qDebug() <<  velocity << size_vehicles;
    velocity_total+= velocity;
}

void Flux(float density)
{
   // qDebug() << "Flujo" << ((float) density * velocity);
    flux = ((float) density * velocity);
    flux_total+= flux;
}

void CalculateSaveMeasures()
{

    save_velocity+= velocity_total / n_ticks;
    save_flux+= flux_total / n_ticks;
}

void SaveMeasures(float density)
{
    save_velocity = save_velocity / n_exp;
    save_flux = save_flux / n_exp;

    //qDebug() << n_exp;

    v_optim = vOptima(density);
    j_optim = jOptima(density);

    fprintf(fp_v, "%f,%f\n", density, save_velocity);//con estos datos se grafica el diagrama fundamental del trafico
    fprintf(fp_f, "%f,%f\n", density, save_flux);//con estos datos se grafica el diagrama fundamental del trafico

    fprintf(fp_vopt, "%f,%f\n", density, v_optim);
    fprintf(fp_fopt, "%f,%f\n", density, j_optim);

}


void SaveMeasuresAutonomous(float density, float per_auto)
{
    save_velocity = save_velocity / n_exp;
    save_flux = save_flux / n_exp;

    //qDebug() << n_exp;
    fprintf(fp_v, "%f,%f,%f\n", density, per_auto, save_velocity);//con estos datos se grafica el diagrama fundamental del trafico
    fprintf(fp_f, "%f,%f,%f\n", density, per_auto, save_flux);//con estos datos se grafica el diagrama fundamental del trafico


}
