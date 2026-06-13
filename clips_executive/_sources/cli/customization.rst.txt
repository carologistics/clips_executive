Customization
#############

The CLI can be customized by overriding key bindings of the available shortcuts, as well as overriding the colors used for display.

.. _cli_log_file_dir:

Log File Directory
++++++++++++++++++

The CLI may store artefacts, such as Log snapshots or file streams or the prompt history into the ROS logging directory by default (controlled by the ``ROS_LOG_DIR`` variable).

Either change this variable or use the ROS parameter ``log_dir`` to change the storage location.

.. _keymap-customization:

Keymap Customization
++++++++++++++++++++

In order to modify the key maps used by the CLI, create a file and pass its path via the ``keymap_overrides`` ROS parameter.

Each line overrides one action.  A chord is written as space-separated
key names; multiple alternative bindings for one action are
comma-separated.

Blank lines and lines starting with ``#`` are ignored.  Only the
actions listed in the file are overridden; all others keep their
defaults.
:kbd:`Space` is specified as "space", other special keys follow the key notation of `prompt_toolkit`_.


The following file contains the default key map:

.. code-block:: ini

   # cx-clips-cli default keymap
   # Copy and modify as needed, then pass via -p keymap_overrides:=<path>

   # Vi-style append
   append              = a

   # INSERT mode
   submit              = enter
   clear_input         = c-c
   accept_suggestion   = c-space, c-f, c-right

   # NORMAL mode — engine control
   pause               = space p
   resume              = space r
   tick                = space t

   # NORMAL mode — UI control
   command_palette     = space m
   select_env          = space e
   toggle_filter       = space f
   open_log            = space l
   toggle_scroll       = space s
   save_log            = space S
   toggle_stream       = space T
   clear_log           = space d
   show_help           = space h
   quit                = space q

   # ON LOG mode
   log_search_fwd      = /
   log_search_bwd      = ?
   log_next_match      = n
   log_prev_match      = N
   log_scroll_up       = pageup
   log_scroll_down     = pagedown
   log_goto_top        = g
   log_goto_bottom     = G
   log_escape          = escape
   log_copy            = c-c

   # Floating selection windows (environment selector, command palette)
   float_up            = up
   float_down          = down
   float_backspace     = backspace
   float_accept        = enter
   float_cancel        = escape, c-c

.. _style-customization:

Style Customization
+++++++++++++++++++

Similarly, the used color palette can be customized by supplying a path to a file via the ``style_overrides`` ROS parameter.

The file supports an optional ``[dark]`` and ``[light]`` section;
entries before any section header apply to both themes.

Values use `prompt_toolkit`_ style strings:
a foreground colour (``#rrggbb``), background (``bg:#rrggbb``), and
modifiers (``bold``, ``italic``, ``underline``) separated by spaces.

The syntax highlighting in the CLI additionally relies on `pygments`_ styles.
You can change the used style by supplying a suitable style name via te ``pygments_style`` ROS parameter.

The following file contains the default style map:

.. code-block:: ini

   # cx-clips-cli default styles
   # Copy and modify as needed, then pass via -p style_overrides:=<path>
   # Use [dark] and [light] sections to target a specific theme.
   # Entries before any section header apply to both themes.

   [dark]
   # Syntax base: github-dark (override with pygments_style parameter)
   bottom-toolbar      = bg:#f5e8d8 fg:#1c1c1c
   # default text
                       = #f8f8f2
   pygments.keyword    = #ff79c6 bold
   pygments.name       = #f8f8f2
   pygments.literal    = #f1fa8c
   pygments.string     = #f1fa8c
   pygments.comment    = #6272a4 italic
   pygments.punctuation = #ff79c6
   pygments.operator   = #ff79c6
   # status bar segments
   bl-separator        = bg:#822659
   bl-text             = bg:#f0f0f0
   bl-highlight        = bg:#2596be
   bl-success          = bg:#3fb950
   bl-failure          = bg:#ff4550
   # log output
   log-prefix          = #888888
   log-error           = #ff4444
   log-warn            = #ffaa00
   # search
   search-prompt       = #00ff00
   search-match        = bg:#ffaa00 #000000 bold

   [light]
   # Syntax base: friendly (override with pygments_style parameter)
   bottom-toolbar      = bg:#484b6a fg:#e4e5f1
   # default text
                       = #212121 fg:#ffffff
   pygments.keyword    = #7c4dff bold
   pygments.name       = #212121
   pygments.literal    = #c75000
   pygments.string     = #c75000
   pygments.comment    = #888888 italic
   pygments.punctuation = #7c4dff
   pygments.operator   = #7c4dff
   # status bar segments
   bl-separator        = bg:#6f4e37
   bl-text             = bg:#000000
   bl-highlight        = bg:#6f42c1
   bl-success          = bg:#1a7f37
   bl-failure          = bg:#b31d28
   # log output
   log-prefix          = #888888
   log-error           = #ff4444
   log-warn            = #ffaa00
   # search
   search-prompt       = #00ff00
   search-match        = bg:#ffaa00 #000000 bold
