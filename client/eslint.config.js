import js from '@eslint/js';
import tseslint from 'typescript-eslint';
import eslintPluginPrettierRecommended from 'eslint-plugin-prettier/recommended';
import globals from 'globals';

export default [
  js.configs.recommended,
  ...tseslint.configs.recommended,
  eslintPluginPrettierRecommended,
  {
    languageOptions: {
      globals: {
        ...globals.browser,
        ...globals.es2021,
      },
    },
    rules: {
      '@typescript-eslint/no-unused-vars': ['warn', { argsIgnorePattern: '^_' }],
    },
  },
  {
    files: ['firm_typescript/examples/**/*.js'],
    languageOptions: {
      globals: globals.node,
    },
  },
  {
    ignores: [
      '.venv/',
      'firm_typescript/pkg/',
      'firm_typescript/typescript/dist/',
      'node_modules/',
      'target/',
    ],
  },
];
