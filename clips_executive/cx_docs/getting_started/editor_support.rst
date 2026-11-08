.. _editor_support:

Editor Support for CLIPS
========================

As CLIPS is a rather specialized language, the tooling around it is limited.

However, we are currently developing a LSP-based editor integration, which you can find here and you can optionally check out if you want:

The `CLIPS Language Server <https://github.com/carologistics/CLIPS-language-server>`_
implements the `Language Server Protocol <https://microsoft.github.io/language-server-protocol/>`_
for CLIPS 6.4.2, so it works with any editor that has an LSP client.

Note that functions and templates that the |CX| plugins register from C++ are not part of plain CLIPS,
so the server reports them as undefined unless it knows about them.
The language server can read a declaration file (``.clpd``) generated from the embedding program,
see the `host-defined names section <https://github.com/carologistics/CLIPS-language-server#features>`_
of its README.
