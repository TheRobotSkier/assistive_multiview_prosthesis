# Added by ForgeCode installer
export PATH="/home/daniel/.local/bin:$PATH"

. "$HOME/.local/bin/env"

# --- Vim mode configuration ---
bindkey -v
export KEYTIMEOUT=1

# Fix Backspace behavior (allow deleting past the insert point)
bindkey '^?' backward-delete-char
bindkey '^H' backward-delete-char

# Set editor for edit-command-line (zle uses $VISUAL or $EDITOR)
export VISUAL="vim"
export EDITOR="vim"

# Vim mode indicator via prompt color shift
# INSERT: Crisp Cyan | NORMAL: Warm Amber | VISUAL: Soft Purple (Much cleaner than dark blue)
_vim_mode_indicator() {
  case $KEYMAP in
    vicmd)
      # NORMAL: warm amber
      _VIM_DIR_COLOR="%F{214}"
      _VIM_CHAR_COLOR="%F{214}"
      ;;
    visual)
      # VISUAL: lavender/purple (highly visible accent)
      _VIM_DIR_COLOR="%F{141}"
      _VIM_CHAR_COLOR="%F{141}"
      ;;
    viins|main|*)
      # INSERT: default bright cyan
      _VIM_DIR_COLOR="%F{81}"
      _VIM_CHAR_COLOR="%F{81}"
      ;;
  esac
  zle reset-prompt
}
zle -N zle-keymap-select _vim_mode_indicator

# Re-evaluate mode on line init
_zle_line_init() {
  _vim_mode_indicator
}
zle -N zle-line-init _zle_line_init

# Reset to insert colors when command is accepted
_zle_line_finish() {
  _VIM_DIR_COLOR="%F{81}"
  _VIM_CHAR_COLOR="%F{81}"
  zle reset-prompt
}
zle -N zle-line-finish _zle_line_finish

# Initialize to insert mode colors
_VIM_DIR_COLOR="%F{81}"
_VIM_CHAR_COLOR="%F{81}"

# --- Custom prompt configuration ---
export NERD_FONT=1

# Prompt layout:  agent_icon  directory  >
setopt PROMPT_SUBST

# Capture last exit status before any commands overwrite it
_last_exit_status() {
    _EXIT_STATUS=$?
}
precmd_functions+=(_last_exit_status)

# Refined Palette:
# %K{71}  = Muted pastel green (Success)
# %K{167} = Muted terracotta red (Failure)
# %K{236} = Sleek dark charcoal block for path/arrow
PROMPT='%(?.%K{71}%F{232}.%K{167}%F{232}) $(_forge_agent_icon) %f%k%K{236}${_VIM_DIR_COLOR}%B %1d %b%f%k%K{236}${_VIM_CHAR_COLOR}%B> %b%f%k '

# >>> forge initialize >>>
# !! Contents within this block are managed by 'forge zsh setup' !!
# !! Do not edit manually - changes will be overwritten !!

# Add required zsh plugins if not already present
if [[ ! " ${plugins[@]} " =~ " zsh-autosuggestions " ]]; then
    plugins+=(zsh-autosuggestions)
fi
if [[ ! " ${plugins[@]} " =~ " zsh-syntax-highlighting " ]]; then
    plugins+=(zsh-syntax-highlighting)
fi

# Load forge shell plugin (commands, completions, keybindings) if not already loaded
if [[ -z "$_FORGE_PLUGIN_LOADED" ]]; then
    eval "$(forge zsh plugin)"
fi

# Load forge shell theme (prompt with AI context) if not already loaded
# We override RPROMPT to keep everything on the left
if [[ -z "$_FORGE_THEME_LOADED" ]]; then
    eval "$(forge zsh theme)"
    RPROMPT=''
fi

# Editor for editing prompts (set during setup)
# To change: update FORGE_EDITOR or remove to use $EDITOR
export FORGE_EDITOR="vim"
# <<< forge initialize <<<

# --- Custom forge prompt: agent icon helper ---
_AGENT_ICONS=(
  forge  ''
  muse   ''
)
_AGENT_ICON_FALLBACK='󰚩'

_forge_agent_icon() {
    local icon="$_AGENT_ICON_FALLBACK"
    local agent="${_FORGE_ACTIVE_AGENT:-forge}"
    local i
    for (( i=1; i<=${#_AGENT_ICONS[@]}; i+=2 )); do
        if [[ "${agent:l}" == "${_AGENT_ICONS[$i]:l}" ]]; then
            icon="${_AGENT_ICONS[$((i+1))]}"
            break
        fi
    done
    echo "$icon"
}
