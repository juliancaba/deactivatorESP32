#ifndef __SERVER_CONFIG_H__
#define __SERVER_CONFIG_H__


const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>HTML Form to Input Data</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  
  <style>
    html {font-family: Sans-Serif; display: inline-block; text-align: left;}
    h3 {font-size: 1.0rem; color: #FFA000;}
  </style>
  
  <script>
    function message_popup() {
      alert("Grabando datos en la EEPROM del procesador");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script>
  
  </head>
  
  <body>
      <h3>Generador de pulsos (Desactivador)</h3>
      <hr size="1px" color="black" />
 <fieldset>
    <legend> Parametros de pulsos </legend>
      <form action="/get" target="hidden-form"><br>
            - Duracion del pulso (valor actual en estado alto: %pulse_size_ms% ms) :&emsp;&emsp;&emsp;<input type="range" name="pulse_size_ms" value="100" step=50 min="50" max="2500" 
            oninput="document.getElementById('fPulso').innerHTML = this.value" />
           <label id="fPulso"></label> ms  
           &emsp;<input type="submit" value="Grabar" onclick="message_popup()">
      </form><br>
      <form action="/get" target="hidden-form">
            - Duracion de la pausa (valor actual en estado bajo: %pause_size_ms% ms) :&emsp;   <input type="range" name="pause_size_ms" value="100" step=50 min="50" max="2000" 
            oninput="document.getElementById('fPause').innerHTML = this.value" />
           <label id="fPause"></label> ms  
           &emsp;<input type="submit" value="Grabar" onclick="message_popup()">
      </form><br>
      <form action="/get" target="hidden-form">
            - Numero de pulsos a enviar (valor actual: %pulse_num%) :&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;<input type="range" name="pulse_num" value="1" step=1 min="1" max="15" 
            oninput="document.getElementById('fTren').innerHTML = this.value" />
           <label id="fTren"></label>
           &emsp;&emsp;<input type="submit" value="Grabar" onclick="message_popup()">
      </form>
   </fieldset><br>
   
   <fieldset>
      <legend> Pulsos </legend>
      <form action="/get" target="hidden-form"><br>
            - Envio de pulsos (valor actual %start_pulse%) :&emsp;&emsp;<input type="range" name="start_pulse" value="0" min="0" step=1 max="1" 
            oninput="document.getElementById('fStartPulsos').innerHTML = this.value" />
           <label id="fStartPulsos"></label>
           &emsp;<input type="submit" value=" start / stop pulsos" onclick="message_popup()">
      </form><br>
   </fieldset><br>
    <hr size="1px" color="black" /><br>     

    <iframe style="display:none" name="hidden-form"></iframe>
  </body>
</html>)rawliteral";


const char* parameter_durPulso = "pulse_size_ms";
const char* parameter_durPausa = "pause_size_ms";
const char* parameter_NumPulsos = "pulse_num";
const char* parameter_StartPulsos = "start_pulse";


#endif