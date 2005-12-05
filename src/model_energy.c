
///////////////////////////////////////////////////////////////////////////
//
// File: model_energy.c
// Author: Richard Vaughan
// Date: 10 June 2004
//
// CVS info:
//  $Source: /home/tcollett/stagecvs/playerstage-cvs/code/stage/src/model_energy.c,v $
//  $Author: rtv $
//  $Revision: 1.26 $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/time.h>
#include <math.h>
#include <assert.h>

//#define DEBUG

#include "stage_internal.h"
#include "gui.h"
extern stg_rtk_fig_t* fig_debug;

#define TIMING 0
#define ENERGY_FILLED 1


int energy_startup( stg_model_t* mod );
int energy_shutdown( stg_model_t* mod );
int energy_update( stg_model_t* mod );

void energy_load( stg_model_t* mod );

int energy_render_data( stg_model_t* mod, char* name, 
			void* data, size_t len, void* userp );
int energy_render_data_text( stg_model_t* mod, char* name, 
			     void* data, size_t len, void* userp );
int energy_unrender_data( stg_model_t* mod, char* name, 
			  void* data, size_t len, void* userp );
int energy_unrender_data_text( stg_model_t* mod, char* name, 
			  void* data, size_t len, void* userp );
int energy_render_cfg( stg_model_t* mod, char* name, 
		       void* data, size_t len, void* userp );

const double STG_ENERGY_PROBE_RANGE_DEFAULT = 1.0;
const double STG_ENERGY_GIVE_DEFAULT = 0.0;
const double STG_ENERGY_TAKE_DEFAULT = 100.0;
const double STG_ENERGY_CAPACITY_DEFAULT = 10000.0;
const double STG_ENERGY_SIZEX_DEFAULT = 0.08;
const double STG_ENERGY_SIZEY_DEFAULT = 0.18;
const double STG_ENERGY_POSEX_DEFAULT = 0.0;
const double STG_ENERGY_POSEY_DEFAULT = 0.0;
const double STG_ENERGY_POSEA_DEFAULT = 0.0;
const int STG_ENERGY_FIDUCIAL_DEFAULT = 255;

typedef struct
{
  stg_model_t* mod;
  GPtrArray* connections;
  stg_watts_t accum;
  stg_watts_t inputwatts;
} stg_energy_model_t;



int energy_init( stg_model_t* mod ) 
{
  mod->f_startup = energy_startup;
  mod->f_shutdown = energy_shutdown;
  mod->f_update = energy_update;
  mod->f_load = energy_load;

    
  // override the default methods
  mod->f_startup = energy_startup;
  mod->f_shutdown = energy_shutdown;
  mod->f_update = energy_update;
  mod->f_load = energy_load;

  // batteries aren't obstacles or sensible to range sensors
  //mod->obstacle_return = FALSE;
  //mod->ranger_return = FALSE;
  //mod->laser_return = LaserTransparent;
  //mod->fiducial_return = STG_ENERGY_FIDUCIAL_DEFAULT;

  mod->watts = 10.0;

  // sensible energy defaults
  stg_geom_t geom;
  geom.pose.x = STG_ENERGY_POSEX_DEFAULT;
  geom.pose.y = STG_ENERGY_POSEY_DEFAULT;
  geom.pose.a = STG_ENERGY_POSEA_DEFAULT;
  geom.size.x = STG_ENERGY_SIZEX_DEFAULT;
  geom.size.y = STG_ENERGY_SIZEY_DEFAULT;
  stg_model_set_geom( mod, &geom );

  // set up config structure
  stg_energy_config_t econf;
  memset(&econf,0,sizeof(econf));  
  econf.probe_range = STG_ENERGY_PROBE_RANGE_DEFAULT;
  econf.give = STG_ENERGY_GIVE_DEFAULT;
  econf.take = STG_ENERGY_TAKE_DEFAULT;
  econf.capacity = STG_ENERGY_CAPACITY_DEFAULT;
  stg_model_set_property( mod, "energy_config", &econf, sizeof(econf));


  // set up initial data structure
  stg_energy_data_t data;
  memset(&data, 0, sizeof(data));
  data.stored = STG_ENERGY_CAPACITY_DEFAULT;
  data.charging = FALSE;
  data.output_joules = 0.0;
  data.input_joules = 0.0;
  data.output_watts = 0.0;
  data.input_watts = 0.0;
  data.range = STG_ENERGY_PROBE_RANGE_DEFAULT;
  //stg_model_set_data( mod, (void*)&data, sizeof(data));
  stg_model_set_property( mod, "energy_data", &data, sizeof(data));

  // set default color
  stg_color_t col = stg_lookup_color( "orange" ); 
  stg_model_set_property( mod, "color", &col, sizeof(col) );
  
  stg_energy_model_t energy;
  energy.mod = mod;
  energy.inputwatts = 0;
  energy.accum = 0;
  energy.connections = g_ptr_array_new();  
  stg_model_set_property( mod, "energy", &energy, sizeof(energy) );

  
  // adds a menu item and associated on-and-off callbacks
  stg_model_add_property_toggles( mod, "energy_data", 
				  energy_render_data, // called when toggled on
				  NULL,
				  energy_unrender_data, // called when toggled off
				  NULL,
				  "energy data bar",
				  TRUE );

  // hmm =- unfortunately adding two toggles for a property doesn't
  // work and I don't have time to debug that right now.

  // adds a menu item and associated on-and-off callbacks
  /* stg_model_add_property_toggles( mod, "energy_data", 
				  energy_render_data_text, // called when toggled on
				  NULL,
				  energy_unrender_data_text, // called when toggled off
				  NULL,
				  "energy data text",
				  TRUE );
  */

  return 0;
}

int energy_startup( stg_model_t* mod )
{
  return 0; // ok
}

int energy_shutdown( stg_model_t* mod )
{
  return 0; // ok
}

// add sink to source's list of power connections
void energy_connect( stg_model_t* source, stg_model_t* sink )
{
  printf( "connecting %s to %s\n", sink->token, source->token );
  
  stg_energy_model_t* en = stg_model_get_property_fixed( source, "energy", sizeof(stg_energy_model_t));
  
  g_ptr_array_add( en->connections, sink );
}

// remove sink from source's list of power connections
void energy_disconnect( stg_model_t* source, stg_model_t* sink )
{
  printf( "disconnecting %s to %s\n", sink->token, source->token );
  
  stg_energy_model_t* en = stg_model_get_property_fixed( source, "energy", sizeof(stg_energy_model_t));
  
  g_ptr_array_remove_fast( en->connections, sink );
}


void energy_load( stg_model_t* mod )
{
  stg_energy_config_t cfg;
  memcpy( &cfg,
	  stg_model_get_property_fixed( mod, "energy_config", 
					sizeof(stg_energy_config_t)),
	  sizeof(cfg));

  cfg.capacity = wf_read_float( mod->id, "capacity", cfg.capacity );
  cfg.probe_range = wf_read_length( mod->id, "range", cfg.probe_range );
  cfg.give = wf_read_float( mod->id, "give", cfg.give );
  cfg.take = wf_read_float( mod->id, "take", cfg.take );
  
  stg_model_set_property( mod, "energy_config", &cfg, sizeof(cfg));
  
  // refill the tank with the new capacity - we start fully gassed up
  stg_energy_data_t data;
  memcpy( &data,
	  stg_model_get_property_fixed( mod, "energy_data", sizeof(data)),
	  sizeof(data));

  data.stored = cfg.capacity;
  stg_model_set_property( mod, "energy_data", &data, sizeof(data));
}

int energy_match( stg_model_t* mod, stg_model_t* hitmod )
{
  // Ignore myself, my children, and my ancestors.
  if( mod != hitmod && (!stg_model_is_related(mod,hitmod))  &&  
      stg_model_get_property_fixed( hitmod, "energy", sizeof(stg_energy_model_t)) )
    return 1;
  
  return 0; // no match
}	


stg_watts_t energy_connection_sum( stg_model_t* mod, GPtrArray* cons )
{
  stg_watts_t watts = 0;
  
  // find the total current required, so I can do current limiting
  if( cons && cons->len )
    for( int i=0; i < cons->len; i++ )
      {
	stg_model_t* con = (stg_model_t*)g_ptr_array_index( cons, i );
	
	if( con == mod ) // skip myself
	  continue;
	
	watts += con->watts;
      }
  
  return watts;
}

int energy_update( stg_model_t* mod )
{     
  PRINT_DEBUG1( "energy service model %d", mod->id  );  
  
  stg_energy_data_t data;
  memcpy( &data, 
	  stg_model_get_property_fixed( mod, "energy_data", sizeof(data) ),
	  sizeof(data));
  
  stg_energy_config_t cfg;
  memcpy( &cfg, 
	  stg_model_get_property_fixed( mod, "energy_config", sizeof(cfg) ),
	  sizeof(cfg));
  
  stg_energy_model_t* en = stg_model_get_property_fixed( mod, "energy", sizeof(stg_energy_model_t));

  // CHARGING - am I connected to something for the next timestep?
  data.charging = FALSE;
  double joules_required = cfg.capacity - data.stored;
  
  //if( joules_required > 0 && cfg.probe_range > 0 )
    {
      stg_pose_t pose;
      stg_model_get_global_pose( mod, &pose );
      
      itl_t* itl = itl_create( pose.x, pose.y, pose.a, cfg.probe_range, 
			       mod->world->matrix, 
			       PointToBearingRange );
      
      stg_model_t* hitmod = 
	itl_first_matching( itl, energy_match, mod );
      
      if( hitmod )
	{
	  printf( "CONNECTING to %s\n", hitmod->token );
	  data.range = itl->range;
	  data.charging = TRUE;
	  energy_connect( hitmod, mod );
	}
    }
  
  // INPUT
  // see how much power is being supplied to me by others this timestep.
  // They will have poked it into my accumulator property
   
  data.input_watts = en->inputwatts;
  
  double input_joules =  data.input_watts * mod->world->sim_interval/1000.0;
  data.input_joules += input_joules;
  data.stored += input_joules;
  
  // clear the accumulator
  en->inputwatts = 0.0;

  // OUTPUT


  if( 1 )
  // if I give output to others and I have some juice to give
  //if( cfg.give > 0  && data.stored > 0 )
    {
      // add up the power required by all connected devices
      stg_watts_t watts = 0.0;
      
      // get the connected devices
      //GPtrArray* cons = stg_model_get_property_fixed( mod, "connections", sizeof(GPtrArray) );
      //if( cons
      watts += energy_connection_sum( mod, en->connections );
      
      // find all the locally connected devices
      GPtrArray* locals = g_ptr_array_new();

     stg_model_tree_to_ptr_array( stg_model_root(mod), locals);
      
      watts += energy_connection_sum( mod, locals );
						       
      //printf( "total watts required %.2f\n", watts ); 
      
      // feed all the connected devices
      
     for( int i=0; i < en->connections->len; i++ )
	{      
	  printf( "handling connection %d\n", i );

	  stg_model_t* con = (stg_model_t*)g_ptr_array_index( en->connections, i );
	  
	  if( con == mod ) // skip myself
	    continue;
	  
	  //	  if( con->watts )
	  //printf( "%s -> %.2fW -> %s\n", 
	  //    mod->token,
	  //    con->watts,
	  //    con->token );

	  
	  // if the connected unit is an energy device, we poke some
	  // energy into it. if it's not an energy device, the energy just
	  // disappears into the entropic void
	  //if( 1 )//con->type == STG_MODEL_ENERGY)
	    {
	      // todo - current limiting
	      stg_energy_config_t* concfg = 
		stg_model_get_property_fixed( con, "energy_config", sizeof(concfg));

	      if( concfg )
		{
		  
		  double watts = concfg->take;
		  
		  //stg_energy_data_t* condata = (stg_energy_data_t*)(con->data);	  
		  //double watts = con->watts;
		  
		  // dump the watts in the model's accumulator
		  stg_energy_model_t* en_con =  
		    stg_model_get_property_fixed( con, "energy", sizeof(stg_energy_model_t) );
		  
		  en_con->inputwatts += watts;
		  
		  //printf( " (stored %.2fW)", watts );
		}
	      
	      
	      //puts( "" );
	    }
	}

      // now disconnect everyone the fast way
      g_ptr_array_set_size( en->connections, 0 );
      
      double joules = watts * mod->world->sim_interval/1000.0;
      joules = MIN( joules, data.stored );
      data.output_watts = watts;
      data.output_joules += joules;
      data.stored -= joules;      
    }
  
  // re-publish our data (so it gets rendered on the screen)
  stg_model_set_property( mod, "energy_data", &data, sizeof(data));

  return 0; // ok
}

int energy_unrender_data( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{ 
  stg_model_fig_clear( mod, "energy_data_fig" );
  stg_model_fig_clear( mod, "energy_data_figx" );
  return 1; // quit callback
}

int energy_unrender_data_text( stg_model_t* mod, char* name, void* data, size_t len, void* userp )
{ 
  stg_model_fig_clear( mod, "energy_data_text_fig" );
  return 1; // quit callback
}

int energy_render_data( stg_model_t* mod,
			 char* name,
			 void* vdata, 
			 size_t len,
			 void* userp )
{
  PRINT_DEBUG( "energy data render" );
  
  stg_rtk_fig_t *fig, *figx;
  
  if( (fig = stg_model_get_fig( mod, "energy_data_fig" )))
    stg_rtk_fig_clear( fig );
  else
    fig = stg_model_fig_create( mod, "energy_data_fig", "top", STG_LAYER_ENERGYDATA );  
  
  if( (figx = stg_model_get_fig( mod, "energy_data_figx" )))
    stg_rtk_fig_clear( figx );
  else
    figx = stg_model_fig_create( mod, "energy_data_figx", "top", 90 );  
  
  // if this model has a energy subscription

  //if(  mod->subs )
  //if(  1 )
  {  
    
    stg_energy_data_t* data =  (stg_energy_data_t*)vdata;
    
    stg_energy_config_t* cfg = 
      stg_model_get_property_fixed( mod, "energy_config", 
				    sizeof(stg_energy_config_t));
    
    stg_geom_t geom;
    stg_model_get_geom( mod, &geom );
    
    double box_height = geom.size.y; 
    double box_width = geom.size.x;
    double fraction = data->stored / cfg->capacity;
    double bar_height = box_height * fraction;
    double bar_width = box_width;
    
    stg_rtk_fig_color_rgb32(figx, 0xFFFFFF ); // white
    
    stg_rtk_fig_rectangle( figx, 
			   0,0,0, 
			   box_width, box_height,
			   TRUE );
    
    if( fraction > 0.5 )
      stg_rtk_fig_color_rgb32(figx, 0x00FF00 ); // green
    else if( fraction > 0.1 )
      stg_rtk_fig_color_rgb32(figx, 0xFFFF00 ); // yellow
    else
      stg_rtk_fig_color_rgb32(figx, 0xFF0000 ); // red      
    
    stg_rtk_fig_rectangle( figx, 
			   0,
			   bar_height/2.0 - box_height/2.0,
			   0, 
			   bar_width,
			   bar_height,
			   TRUE );
    
    stg_rtk_fig_color_rgb32(figx, 0x0 ); // black
    
    stg_rtk_fig_rectangle( figx, 
			   0,
			   bar_height/2.0 - box_height/2.0,
			   0, 
			   bar_width,
			   bar_height,
			   FALSE );
    
    stg_rtk_fig_rectangle( figx, 
			   0,0,0, 
			   box_width,
			   box_height,
			   FALSE );
    
    //stg_rtk_fig_color_rgb32(fig, stg_lookup_color(STG_ENERGY_COLOR) );
    
    // if( data->charging ) 
    {
      stg_rtk_fig_color_rgb32(figx, 0x00BB00 ); // green
      stg_rtk_fig_arrow_fancy( figx, 0,0,0, data->range, 0.25, 0.10, 1 );
      
      stg_rtk_fig_color_rgb32(figx, 0 ); // black
      stg_rtk_fig_arrow_fancy( figx, 0,0,0, data->range, 0.25, 0.10, 0 );
    }
    
    /*       if( 1 ) */
    /*       //if( cfg->capacity > 0 ) */
    /* 	{ */
    
    char buf[256];
    snprintf( buf, 128, "%.0f/%.0fJ (%.0f%%)\noutput %.2fW %.2fJ\ninput %.2fW %.2fJ\n%s",
	      data->stored,
	      cfg->capacity,
	      data->stored/cfg->capacity * 100,
	      data->output_watts,
	      data->output_joules,
	      data->input_watts,
	      data->input_joules,
	      data->charging ? "charging" : "" );
    
    stg_rtk_fig_text( figx, 0.6,0.0,0, buf );
    
    /* 	} */
    //else
    //{
    //char buf[128];
    //snprintf( buf, 128, "mains supply\noutput %.2fW\n", 
    //    data->output );
    
    // stg_rtk_fig_text( fig, 0.6,0,0, buf ); 	  
    //}
    
  }
  
  return 0;
}

int energy_render_data_text( stg_model_t* mod,
			     char* name,
			     void* vdata, 
			     size_t len,
			     void* userp )
{
  stg_rtk_fig_t* fig = stg_model_get_fig( mod, "energy_data_text_fig" );  
  
  if( ! fig )
    fig = stg_model_fig_create( mod, "energy_data_text_fig", "top", STG_LAYER_ENERGYDATA );  
  else
    stg_rtk_fig_clear( fig );
  
  
  stg_energy_data_t* data = (stg_energy_data_t*)vdata;

  stg_energy_config_t* cfg = 
    stg_model_get_property_fixed( mod, "energy_config", 
				  sizeof(stg_energy_config_t));
  
  char buf[256];
  snprintf( buf, 128, "%.0f/%.0fJ (%.0f%%)\noutput %.2fW %.2fJ\ninput %.2fW %.2fJ\n%s", 
	    data->stored, 
	    cfg->capacity, 
	    data->stored/cfg->capacity * 100,
	    data->output_watts,
	    data->output_joules,
	    data->input_watts,
	    data->input_joules,
	    data->charging ? "charging" : "" );
  
  stg_rtk_fig_text( fig, 0.6,0.0,0, buf ); 
  
  //else
  //{
  //char buf[128];
  //snprintf( buf, 128, "mains supply\noutput %.2fW\n", 
  //    data->output );
  
  // stg_rtk_fig_text( fig, 0.6,0,0, buf ); 	  
  //

  return 0;
}

/* void energy_render_config( stg_model_t* mod ) */
/* {  */
/*   PRINT_DEBUG( "energy config render" ); */
  
  
/*   if( mod->gui.cfg  ) */
/*     stg_rtk_fig_clear(mod->gui.cfg); */
/*   else // create the figure, store it in the model and keep a local pointer */
/*     { */
/*       mod->gui.cfg = stg_rtk_fig_create( mod->world->win->canvas,  */
/* 				   mod->gui.top, STG_LAYER_ENERGYCONFIG ); */

/*       stg_rtk_fig_color_rgb32( mod->gui.cfg, stg_lookup_color( STG_ENERGY_CFG_COLOR )); */
/*     } */

/*   stg_energy_config_t cfg; */
/*   stg_model_get_config(mod, &cfg, sizeof(cfg) ); */

/*   if( cfg.take > 0 ) */
/*     //stg_rtk_fig_arrow_fancy( mod->gui.cfg, 0,0,0, cfg.probe_range, 0.25, 0.10, 1 ); */
/*     stg_rtk_fig_arrow( mod->gui.cfg, 0,0,0, cfg.probe_range, 0.25  ); */
  
/* } */


