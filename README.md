# Integracja algorytmów sterowania robotem mobilnym w realizacji zadan nawigacyjnych z wykorzystaniem lokalnej kamery  
Tematem niniejszej pracy jest stworzenie oprogramowania dla dwukołowego robota o napedzie róznicowym MTracker, które bedzie pozwalac na wykrycie przeszkody oraz jej bezpieczne ominiecie. Cel pracy obejmuje swoim zagadnieniem nastepujacy zakres:  
1. Zapoznanie sie z literatura przedmiotu, platforma sprzetowa, oprogramowaniem systemowym i oprogramowaniem do przetwarzania obrazu.
2. Implementacja oprogramowania realizujacego sledzenie trajektorii w srodowisku ROS.
3. Wykrywanie przeszkód z wykorzystaniem lokalnej kamery i markerów.
4. Realizacja zadania sledzenia trajektorii z omijaniem przeszkód przez miniaturowy dwukołowy robot mobilny.
5. Opracowanie wyników ekesperymentów ruchowych wraz z dokumentacja projektu.  

**Struktura programu**:
Praca całego oprogramowania oparta jest na pieciu programach, komunikacja miedzy nimi została ukazana w formie grafu na poniższym rysunku.   
![Screenshot](photos/struktura.PNG)  
Nodelet_manager wraz z ueye_cam_nodelet odpowiedzialne sa za wysyłanie aktualnego obrazu. Image_processing pełni trzy funkcje: odbiera obraz, wykonuje funkcje zwiazane z wykryciem markera oraz wysyła współrzedne markera do programu Controller. Ten program wykorzystuje dane odebrane od programu image_processing, do poprawnego omijania przeszkód. Controller
odbiera dane dotyczace aktualnej pozycji robota, generuje równiez trajektorie zadana oraz wyznacza aktualny sygnał sterujacy, który jest wysyłany do programu Mtracker. Program ten ma za zadanie: wysyłac sygnał sterujacy do sterowników mocy zasilajacych koła robota oraz wysyłac dane dotyczace odometrii. Całosc została podsumowana w poniższej tabeli:    
![Screenshot](photos/tabela.PNG)  

**Schemat algorytmu wykrywaniu przeszkód**:  
![Screenshot](photos/algorytm.PNG)  

**Procedura ominięcia przeszkody**:  
![Screenshot](photos/ominiecie.PNG)  
 
**Przykładowy wykres ominięcia trzech przeszkód - raz ze strony prawej i dwa razy z lewej:**  
![Screenshot](photos/wykres.PNG)  

**Film ukazujący działanie w rzeczywistości:**  
[![Film](photos/film.mp4)](photos/film.mp4)  

W repozytorium znajdują się tylko pliki wykonywalne wykonane w ramach pracy inżynierkiej, nie dołączam całego projektu, gdyż pliki w nim wykorzystywane pochodziły z rożnych źródeł.

