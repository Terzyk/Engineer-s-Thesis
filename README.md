# Integracja algorytmów sterowania robotem mobilnym w realizacji zadan nawigacyjnych z wykorzystaniem lokalnej kamery  
Tematem niniejszej pracy jest stworzenie oprogramowania dla dwukołowego robota o napedzie róznicowym MTracker, które bedzie pozwalac na wykrycie przeszkody oraz jej bezpieczne ominiecie. Cel pracy obejmuje swoim zagadnieniem nastepujacy zakres:  
1. Zapoznanie sie z literatura przedmiotu, platforma sprzetowa, oprogramowaniem systemowym i oprogramowaniem do przetwarzania obrazu.
2. Implementacja oprogramowania realizujacego sledzenie trajektorii w srodowisku ROS.
3. Wykrywanie przeszkód z wykorzystaniem lokalnej kamery i markerów.
4. Realizacja zadania sledzenia trajektorii z omijaniem przeszkód przez miniaturowy dwukołowy robot mobilny.
5. Opracowanie wyników ekesperymentów ruchowych wraz z dokumentacja projektu.  

**Struktura programu**:
Praca całego oprogramowania oparta jest na pieciu programach, komunikacja miedzy nimi została ukazana w formie grafu na poniższym rysunku.   
<p align="center">
  <img width="460" height="300" src=photos/struktura.PNG>
</p>
 
Nodelet_manager wraz z ueye_cam_nodelet odpowiedzialne sa za wysyłanie aktualnego obrazu. Image_processing pełni trzy funkcje: odbiera obraz, wykonuje funkcje zwiazane z wykryciem markera oraz wysyła współrzedne markera do programu Controller. Ten program wykorzystuje dane odebrane od programu image_processing, do poprawnego omijania przeszkód. Controller
odbiera dane dotyczace aktualnej pozycji robota, generuje równiez trajektorie zadana oraz wyznacza aktualny sygnał sterujacy, który jest wysyłany do programu Mtracker. Program ten ma za zadanie: wysyłac sygnał sterujacy do sterowników mocy zasilajacych koła robota oraz wysyłac dane dotyczace odometrii. Całosc została podsumowana w poniższej tabeli:    
<p align="center">
  <img width="460" height="300" src=photos/tabela.PNG>
</p>
 
