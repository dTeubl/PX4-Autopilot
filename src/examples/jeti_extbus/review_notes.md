10.01.2025
----------

+ Use spaces instead of tabs. Most tools will work better that way, and increases reasbility
+ Include module visibility -> Tell me to explain or find a good article about it.
    + Visibility matters: **extbus.h** only has the *<vector>*, nothing else, but it needs types as well from *<cstdint>*. There is a nice and clean hierarchy there from module visibility point of view.

+ Try to order the content of the file by interface priority. Put high level interface function on top, which are only called via external modules. Put the internaly called one below those. It will help readers/editors to avoid unecessery jumps in the file during editing/reading.

+ Have a **TODO** list, either inside the main module, or just as a text file next to the modules. Maintain that all the time. Add new elements on the fly, and mark finished items.

+ GetChannelValues(...) seems a bit overkill at the moment. Likely there will be no need for additional storage for the values, since they'll be copied early on into their data structure. Decoding/removing siingle channel values by index, probably enough for now. Keep **KISS** and **YAGNI** in mind all the time. Don't use more resource than you need, specially if you don't know if you'll need it or not. Removing code is easy, putting in is hard, not putting code in is even easier.
